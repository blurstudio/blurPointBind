#ifdef GPU
#include "blurPointBind.h"
#include "blurPointBindGPU.h"

#include <vector>
#include <algorithm>
#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>

MString PointBindGPUDeformer::pluginLoadPath;

MGPUDeformerRegistrationInfo* PointBindGPUDeformer::getGPUDeformerInfo() {
    static PointBindGPUDeformerInfo theOne;
    return &theOne;
}

bool PointBindGPUDeformer::validateNodeInGraph(
    MDataBlock& block, const MEvaluationNode& evalNode,
    const MPlug& plug, MStringArray* messages
) {
    // PointBindGPUDeformer supports everything on the PointBindDeformer node
    return true;
}

bool PointBindGPUDeformer::validateNodeValues(
    MDataBlock& block, const MEvaluationNode& evalNode,
    const MPlug& plug, MStringArray* messages
) {
    // PointBindGPUDeformer supports everything on the PointBindDeformer node
    return true;
}

MPxGPUDeformer::DeformerStatus PointBindGPUDeformer::evaluate(
    MDataBlock& block,
    const MEvaluationNode& evalNode,
    const MPlug& outPlug,
    unsigned int numElements,
    const MAutoCLMem inputBuffer,
    const MAutoCLEvent inputEvent,
    MAutoCLMem outputBuffer,
    MAutoCLEvent& outputEvent)
{
    // I need to transfer any data I care about onto the GPU, and
    // I need to run my OpenCL Kernel. First, transfer the data.
    // PointBindDeformer only has to transfer the weight array to the GPU
    // I don't need to transfer down the input position buffer,
    // that is already handled by the deformer evaluator, the points are in inputBuffer.
    MObject node = outPlug.node();
    extractArrays(block, evalNode, outPlug, numElements);

    // Now that all the data we care about is on the GPU, setup and run the OpenCL Kernel
    if (!fKernel.get())  {
        // Load the OpenCL kernel if we haven't yet.
        MString openCLKernelFile(pluginLoadPath);
        openCLKernelFile += "/blurPointBind.cl";
        fKernel = MOpenCLInfo::getOpenCLKernel(openCLKernelFile, "blurPointBindKernel");
        if (fKernel.isNull())  {
            std::cerr << "Could not compile kernel: " << openCLKernelFile << "\n";
            return MPxGPUDeformer::kDeformerFailure;
        }
    }

    MStatus status;
    float envelope = block.inputValue(MPxDeformerNode::envelope, &status).asFloat();
    if (!status) return MPxGPUDeformer::kDeformerFailure;

    float pushLen = block.inputValue(PointBindDeformer::aPointBindLength, &status).asFloat();
    if (!status) return MPxGPUDeformer::kDeformerFailure;

    cl_int err = CL_SUCCESS;

    // Set all of our kernel parameters.  Input buffer and output buffer may be changing every frame
    // so always set them.
    unsigned int parameterId = 0;
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)outputBuffer.getReadOnlyRef());
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)inputBuffer.getReadOnlyRef());
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)fCLNormals.getReadOnlyRef());
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)fCLWeights.getReadOnlyRef());
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_float), (void*)&envelope);
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_float), (void*)&pushLen);
    MOpenCLInfo::checkCLErrorStatus(err);
    err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_uint), (void*)&numElements);
    MOpenCLInfo::checkCLErrorStatus(err);

    // Figure out a good work group size for our kernel.
    size_t workGroupSize;
    size_t retSize;
    err = clGetKernelWorkGroupInfo(
        fKernel.get(),
        MOpenCLInfo::getOpenCLDeviceId(),
        CL_KERNEL_WORK_GROUP_SIZE,
        sizeof(size_t),
        &workGroupSize,
        &retSize
    );
    MOpenCLInfo::checkCLErrorStatus(err);

    size_t localWorkSize = 256;
    if (retSize > 0){
        localWorkSize = workGroupSize;
    }
    // global work size must be a multiple of localWorkSize
    size_t globalWorkSize = (localWorkSize - numElements % localWorkSize) + numElements;

    // set up our input events.  The input event could be NULL, in that case we need to pass
    // slightly different parameters into clEnqueueNDRangeKernel
    unsigned int numInputEvents = 0;
    if (inputEvent.get()) {
        numInputEvents = 1;
    }

    // run the kernel
    err = clEnqueueNDRangeKernel(
        MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
        fKernel.get(),
        1,
        NULL,
        &globalWorkSize,
        &localWorkSize,
        numInputEvents,
        numInputEvents ? inputEvent.getReadOnlyRef() : 0,
        outputEvent.getReferenceForAssignment()
    );
    MOpenCLInfo::checkCLErrorStatus(err);

    return MPxGPUDeformer::kDeformerSuccess;
}

void PointBindGPUDeformer::terminate() {
    fCLWeights.reset();
    fCLNormals.reset();
    MOpenCLInfo::releaseOpenCLKernel(fKernel);
    fKernel.reset();
}

bool PointBindGPUDeformer::getWeights(
	MDataBlock& block,
	const MEvaluationNode& evalNode,
	const MPlug& plug,
	float * temp,
	unsigned int pointCount
){
    // Two possibilities: we could have a sparse array in weightList[multiIndex]
    // or there could be nothing in weightList[multiIndex].
    // if nothing is there then all the weights at 1.0f.
    // Get a handle to the weight array we want.
	MStatus status;
	size_t idx = 0;

    MArrayDataHandle weightList = block.outputArrayValue(MPxDeformerNode::weightList, &status);
    if (!status) return false; // we should always be able to get a weightList
    status = weightList.jumpToElement(plug.logicalIndex());
    // it is possible that the jumpToElement fails.  In that case all weights are 1.
    if (!status) {
        for(unsigned int i=0; i<pointCount; i++){
			temp[idx++] = 1.0f;
        }
    }
    else {
        MDataHandle weightsStructure = weightList.inputValue(&status);
        if (!status) return false;
        MArrayDataHandle weights = weightsStructure.child(MPxDeformerNode::weights);
        if (!status) return false;

        // number of non-zero weights
        unsigned int numWeights = weights.elementCount(&status);
        if (!status) return false;

        // we're building a list with a weight per vertex, even if the weight is zero
        unsigned int weightIndex = 0;
        for(unsigned int i=0; i<numWeights; i++, weights.next()) {
            unsigned int weightsElementIndex = weights.elementIndex(&status);
            while (weightIndex < weightsElementIndex) {
				// weights could be sparse, fill in zero weight if no data
				temp[idx++] = 0.0f;
                weightIndex++;
            }
            MDataHandle value = weights.inputValue(&status);
			temp[idx++] = value.asFloat();
            weightIndex++;
        }
        // now we have written the last non-zero weight into temp,
        // but the last non-zero weight doesn't have to be for the
        // last vertex in the buffer.  Add more zero values if necessary.
        while (weightIndex < pointCount) {
			// weights could be sparse, fill in zero weight if no data
			temp[idx++] = 0.0f;
            weightIndex++;
        }
    }
	return true;
}


cl_int EnqueueBuffer(MAutoCLMem& mclMem, size_t bufferSize, void* data) {
    // Convenience function to copy array data to the gpu.
    cl_int err = CL_SUCCESS;
    if (!mclMem.get())	{
        // The buffer doesn't exist yet so create it and copy the data over.
        mclMem.attach(clCreateBuffer(MOpenCLInfo::getOpenCLContext(),
                    CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
                    bufferSize, data, &err));
    }	else {
        // The buffer already exists so just copy the data over.
        err = clEnqueueWriteBuffer(MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
                mclMem.get(), CL_TRUE, 0, bufferSize,
                data, 0, NULL, NULL);
    }
    return err;
}

void PointBindGPUDeformer::extractArrays(
        MDataBlock& block,
        const MEvaluationNode& evalNode,
        const MPlug& plug,
		unsigned int pointCount
) {
    // Note that right now dirtyPlugExists takes an attribute, so
    // if any element in the multi is changing we think it is dirty...
    // To avoid false dirty issues here you'd need to only use one
    // element of the MPxDeformerNode::input multi attribute for each
    // PointBindDeformer node.
    MStatus status;
	/*
    auto weightDpe = evalNode.dirtyPlugExists(MPxDeformerNode::weightList, &status);
    if (!status) return;
    auto inDpe = evalNode.dirtyPlugExists(MPxDeformerNode::input, &status);
    if (!status) return;
    auto lenDpe = evalNode.dirtyPlugExists(PointBindDeformer::aPointBindLength, &status);
    if (!status) return;
    if (fCLWeights.get() && !weightDpe && !inDpe && !lenDpe) return;
	*/

	// get the input mesh corresponding to this output
	MObject node = plug.node();
	unsigned int multiIndex = plug.logicalIndex();
	MPlug inPlug(node, PointBindDeformer::input);
	inPlug.selectAncestorLogicalIndex(multiIndex, PointBindDeformer::input);
	MDataHandle hInput = block.inputValue(inPlug);
	MObject oMesh = hInput.asMesh();
	MFnMesh fnMesh(oMesh, &status);

	// Get the normals
    MFloatVectorArray mNormals;
    mNormals.setLength(pointCount);
    fnMesh.getVertexNormals(false, mNormals);
	float(*normals)[3] = new float[pointCount][3];
	mNormals.get(normals);

	// Get the weights
	float* weights = new float[pointCount];
    bool good = getWeights(block, evalNode, plug, weights, pointCount);

    cl_int err = CL_SUCCESS;
    err = EnqueueBuffer(fCLNormals, pointCount * 3 * sizeof(float), (void*)&normals[0][0]);
    err = EnqueueBuffer(fCLWeights, pointCount * sizeof(float), (void*)weights);

	delete[] weights;
	delete[] normals;
}
#endif
