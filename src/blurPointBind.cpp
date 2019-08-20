#include "blurPointBind.h"

#include <vector>
#include <algorithm>
#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MPointArray.h>

MTypeId PointBindDeformer::id(0x00122703);

MObject PointBindDeformer::aTargetGroup;

MObject PointBindDeformer::aIndices;
MObject PointBindDeformer::aIndexTargets;
MObject PointBindDeformer::aTargets;


void* PointBindDeformer::creator() { return new PointBindDeformer(); }
MStatus PointBindDeformer::initialize() {
	MStatus stat;
	MFnTypedAttribute tAttr;
	MFnCompoundAttribute cAttr;

	// The indices to map to
    aIndices = tAttr.create("indexes", "idx", MFnData::kIntArray, &stat);
	if (!stat) return stat;

	// The target per index
    aIndexTargets = tAttr.create("indexTargets", "itr", MFnData::kIntArray, &stat);
	if (!stat) return stat;

	// The number of targets per vert
    aIndexCounts = tAttr.create("indexTargets", "itr", MFnData::kIntArray, &stat);
	if (!stat) return stat;

	// The meshes I'm binding to
    aTargets = tAttr.create("target", "tar", MFnData::kMesh, &stat);
	if (!stat) return stat;
    tAttr.setArray(true);

	aTargetGroup = cAttr.create("targetGroup", "targetGroup");
	cAttr.setArray(true);
	cAttr.addChild(aIndices);
	cAttr.addChild(aIndexTargets);
	cAttr.addChild(aIndexCounts);
	cAttr.addChild(aTargets);
	addAttribute(aTargetGroup);

	stat = attributeAffects(aTargets, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndexTargets, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndexCounts, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndices, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aTargetGroup, outputGeom);
	if (!stat) return stat;

	//MGlobal::executeCommand("makePaintable -attrType \"multiFloat\" -sm \"deformer\" \"" DEFORMER_NAME "\" \"weights\";");
	return MStatus::kSuccess;
}


MStatus PointBindDeformer::deform(
    MDataBlock& data, MItGeometry& iter,
    const MMatrix& m, unsigned int multiIndex
) {
	MStatus stat;

    // Envelope data from the base deformer class.
    MDataHandle envData = data.inputValue(envelope, &stat);
	if (!stat) return stat;
    float env = envData.asFloat();
	if (env == 0.0f) return stat;

	MArrayDataHandle hTargetGroupArray = data.inputArrayValue(aTargetGroup);
	stat = hTargetGroupArray.jumpToElement(multiIndex);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MDataHandle hTargetGroupData = hTargetGroupArray.inputValue();

    // Get the Counts
	MDataHandle hIndexCounts = hTargetGroupData.child(aIndexCounts);
    MObject oIndexCounts = hIndexCounts.data();
    MFnIntArrayData fnIndexCounts(oIndexCounts, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	MIntArray indexCounts = fnIndexCounts.array();

    // Get the Target indices
	MDataHandle hIndexTargets = hTargetGroupData.child(aIndexTargets);
    MObject oIndexTargets = hIndexTargets.data();
    MFnIntArrayData fnIndexTargets(oIndexTargets, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	MIntArray indexTargets = fnIndexTargets.array();

    // Get the Vertex indices
	MDataHandle hIndices = hTargetGroupData.child(aIndices);
    MObject oIndices = hIndices.data();
    MFnIntArrayData fnIndices(oIndices, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);
	MIntArray indices = fnIndices.array();

	// Get the meshes
	std::vector<int> remap;
	std::vector<MPointArray> pointStack;
	MArrayDataHandle hTarget = hTargetGroupData.child(aTargets);
	if (hTarget.elementCount() == 0) return stat;
	
	int idx = 0;
	hTarget.jumpToElement(0); // Jump to the first *filled* element
	do {
		int ei = hTarget.elementIndex();
		if (ei > remap.size()) remap.resize(ei, -1);
		remap[ei] = idx++;

		MObject mesh = hTarget.inputValue().asMesh();
		MFnMesh meshFn(mesh);
		MPointArray pts;
		meshFn.getPoints(pts);
		pointStack.push_back(pts);
	} while (hTarget.next());

	int prevIdx = 0;
	for (int i = 0; !iter.isDone() && (i < indices.length()); iter.next(), ++i) {
		// Add up and average all the point bind info
		MPoint tar;
		int curIdx = indexCounts[i];
		int count = curIdx - prevIdx;
		for (int id=prevIdx; id < curIdx; ++id){
			int meshIdx = remap[indexTargets[id]];
			if (meshIdx == -1) {count--; continue;}
			tar += pointStack[meshIdx][indices[id]];
		}
		if (count < 1) continue;
		tar = (MVector)tar / count;
		prevIdx = curIdx;

		// Apply the envelope and the weights
        MPoint pt = iter.position();
        float w = weightValue(data, multiIndex, iter.index()) * env;
		pt += (tar - pt) * w;
        iter.setPosition(pt);
	}
    return stat;
}





