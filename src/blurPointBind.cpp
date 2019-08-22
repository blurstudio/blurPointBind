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
MObject PointBindDeformer::aIndexCounts;
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
    aIndexCounts = tAttr.create("indexCounts", "icn", MFnData::kIntArray, &stat);
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

	stat = attributeAffects(aTargetGroup, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aTargets, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndexTargets, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndexCounts, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aIndices, outputGeom);
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


	MArrayDataHandle hTargetGroupArray = data.inputArrayValue(aTargetGroup, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	unsigned inCount = hTargetGroupArray.elementCount(&stat);
	if (inCount == 0) return stat;
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	stat = hTargetGroupArray.jumpToArrayElement(multiIndex);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MDataHandle hTargetGroupData = hTargetGroupArray.inputValue(&stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

    // Get the Counts
	MDataHandle hIndexCounts = hTargetGroupData.child(aIndexCounts);
    MObject oIndexCounts = hIndexCounts.data();
	MFnIntArrayData fnIndexCounts;
	if (oIndexCounts.isNull()) { fnIndexCounts.create(&stat); }
	else { stat = fnIndexCounts.setObject(oIndexCounts); }
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MIntArray indexCounts = fnIndexCounts.array();

    // Get the Target indices
	MDataHandle hIndexTargets = hTargetGroupData.child(aIndexTargets);
    MObject oIndexTargets = hIndexTargets.data();
	MFnIntArrayData fnIndexTargets;
	if (oIndexTargets.isNull()) { fnIndexTargets.create(&stat); }
	else { stat = fnIndexTargets.setObject(oIndexTargets); }
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MIntArray indexTargets = fnIndexTargets.array();

    // Get the Vertex indices
	MDataHandle hIndices = hTargetGroupData.child(aIndices);
    MObject oIndices = hIndices.data();
	MFnIntArrayData fnIndices;
	if (oIndices.isNull()) { fnIndices.create(&stat); }
	else { stat = fnIndices.setObject(oIndices); }
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
		if (ei+1 > remap.size()) remap.resize(ei+1, -1);
		remap[ei] = idx++;

		MObject mesh = hTarget.inputValue().asMesh();
		MFnMesh meshFn(mesh);
		MPointArray pts;
		meshFn.getPoints(pts);
		pointStack.push_back(pts);
	} while (hTarget.next());





	// Test values
	indexCounts.append(0); // starter index // TODO: Automatically add this value
	indexCounts.append(0); // vert index 0
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(1); indices.append(0); // vert index 3
	indexCounts.append(2); indices.append(1);
	indexCounts.append(3); indices.append(2);
	indexCounts.append(4); indices.append(3);
	indexCounts.append(5); indices.append(4);
	indexCounts.append(6); indices.append(5);
	indexCounts.append(7); indices.append(6);
	indexCounts.append(8); indices.append(7);
	indexCounts.append(9); indices.append(8);
	indexCounts.append(10); indices.append(9);
	indexCounts.append(11); indices.append(10);
	indexCounts.append(12); indices.append(11);
	indexCounts.append(13); indices.append(12);
	indexCounts.append(14); indices.append(13);
	indexCounts.append(15); indices.append(14); // vert index 17
	indexCounts.append(15);
	indexCounts.append(15);
	indexCounts.append(15);
	indexCounts.append(15);
	indexCounts.append(15);
	indexCounts.append(15);
	indexCounts.append(16); indices.append(15);
	indexCounts.append(17); indices.append(16);

	for (int t = 0; t < indices.length(); ++t) {
		indexTargets.append(0);
	}



	// Finally do the deformation
	unsigned i = 1, ptr = 0;
	for (; !iter.isDone(); iter.next(), ++i) {
		unsigned ic = indexCounts[i] - indexCounts[i - 1];
		if (ic == 0) continue;
		// TODO: Deal with spaces
		MPoint tar;
		for (; (ptr < indexCounts[i]) && (ptr < indices.length()); ++ptr) {
			unsigned meshIdx = indexTargets[ptr];
			tar += pointStack[remap[meshIdx]][indices[ptr]];
		}

		tar = (MVector)tar / ic;
		MPoint pt = iter.position();
        float w = weightValue(data, multiIndex, iter.index()) * env;
		pt += (tar - pt) * w;
        iter.setPosition(pt);
		if (ptr >= indices.length()) break;
	}




    return stat;
}

