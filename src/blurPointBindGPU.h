#ifdef GPU
#pragma once
#include <maya/MPxDeformerNode.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MMatrix.h>
#include <maya/MDagModifier.h>
#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <vector>

#include "blurPointBind.h"



class PointBindGPUDeformer : public MPxGPUDeformer
{
public:
    // Virtual methods from MPxGPUDeformer
	PointBindGPUDeformer() {};
	virtual ~PointBindGPUDeformer() { terminate(); };

    virtual MPxGPUDeformer::DeformerStatus evaluate(
        MDataBlock& block, const MEvaluationNode&, const MPlug& plug,
        unsigned int numElements, const MAutoCLMem, const MAutoCLEvent, MAutoCLMem, MAutoCLEvent&
    );
    virtual void terminate();

    static MGPUDeformerRegistrationInfo* getGPUDeformerInfo();
    static bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);
    static bool validateNodeValues(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);
    static MString pluginLoadPath;

private:
    // helper methods
    void extractArrays(MDataBlock& block, const MEvaluationNode& evalNode, const MPlug& plug, unsigned int);
	bool PointBindGPUDeformer::getWeights(MDataBlock&, const MEvaluationNode&, const MPlug&, float*, unsigned int);

    // Storage for data on the GPU
    MAutoCLMem fCLWeights;
    MAutoCLMem fCLIndices;
    unsigned int fNumElements;

    // Kernel
    MAutoCLKernel fKernel;
};

class PointBindGPUDeformerInfo : public MGPUDeformerRegistrationInfo
{
public:
    PointBindGPUDeformerInfo(){}
    virtual ~PointBindGPUDeformerInfo(){}
    virtual MPxGPUDeformer* createGPUDeformer() {return new PointBindGPUDeformer(); }

    virtual bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode& evalNode, const MPlug& plug, MStringArray* messages) {
        return PointBindGPUDeformer::validateNodeInGraph(block, evalNode, plug, messages);
    }

    virtual bool validateNodeValues(MDataBlock& block, const MEvaluationNode& evalNode, const MPlug& plug, MStringArray* messages) {
        return PointBindGPUDeformer::validateNodeValues(block, evalNode, plug, messages);
    }
};

#endif
