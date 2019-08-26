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
MObject PointBindDeformer::aTargetMesh;
MObject PointBindDeformer::aTargetWorld;


void* PointBindDeformer::creator() { return new PointBindDeformer(); }

MStatus PointBindDeformer::initialize() {
	MStatus stat;
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
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
    aTargetMesh = tAttr.create("targetMesh", "tam", MFnData::kMesh, &stat);
	if (!stat) return stat;

	// Whether to bind each mesh in worldspace
	aTargetWorld = nAttr.create("targetWorld", "taw", MFnNumericData::kBoolean, false, &stat);
	if (!stat) return stat;

	aTargets = cAttr.create("targets", "targets");
	cAttr.setArray(true);
	cAttr.addChild(aTargetMesh);
	cAttr.addChild(aTargetWorld);

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
	stat = attributeAffects(aTargetMesh, outputGeom);
	if (!stat) return stat;
	stat = attributeAffects(aTargetWorld, outputGeom);
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

	MArrayDataHandle hTargets = hTargetGroupData.child(aTargets);
	unsigned cnxMeshes = hTargets.elementCount();

	std::vector<MPointArray> pointStack;
	std::vector<std::vector<MPoint>> backStack;
	std::vector<bool> worldStack;

	if (cnxMeshes == 0) return stat;
	for (unsigned idx = 0; idx < cnxMeshes; ++idx) {
		hTargets.jumpToElement(idx);
		int ei = hTargets.elementIndex();
		MDataHandle hTarget = hTargets.inputValue();

		MDataHandle hTarWorld = hTarget.child(aTargetWorld);
		bool world = hTarWorld.asBool();
		MSpace::Space space = (world) ? MSpace::kWorld : MSpace::kObject;

		MDataHandle hTarMesh = hTarget.child(aTargetMesh);
		MObject mesh = hTarMesh.asMesh();
		if (mesh.isNull())
			continue;

		// Wait 'till after the check to allocate
		worldStack.resize(ei + 1);
		pointStack.resize(ei + 1);
		backStack.resize(ei + 1);
		worldStack[ei] = world;

		MFnMesh meshFn(mesh);
		MPointArray pts;
		meshFn.getPoints(pts, space);
		pointStack[ei] = pts;
	}

	if (pointStack.size() == 0)
		return stat;
	




	// Test values
	indexCounts.append(0); // vert index 0
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(1); indices.append(0); indexTargets.append(0);
	indexCounts.append(1); indices.append(1); indexTargets.append(0);
	indexCounts.append(1); indices.append(2); indexTargets.append(0);
	indexCounts.append(1); indices.append(3); indexTargets.append(0);
	indexCounts.append(1); indices.append(4); indexTargets.append(0);
	indexCounts.append(1); indices.append(5); indexTargets.append(0);
	indexCounts.append(1); indices.append(6); indexTargets.append(0);
	indexCounts.append(1); indices.append(7); indexTargets.append(0);
	indexCounts.append(1); indices.append(8); indexTargets.append(0);
	indexCounts.append(1); indices.append(9); indexTargets.append(0);
	indexCounts.append(1); indices.append(10); indexTargets.append(0);
	indexCounts.append(1); indices.append(11); indexTargets.append(0);
	indexCounts.append(1); indices.append(12); indexTargets.append(0);
	indexCounts.append(1); indices.append(13); indexTargets.append(0);
	indexCounts.append(1); indices.append(14); indexTargets.append(0);
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(0);
	indexCounts.append(1); indices.append(15); indexTargets.append(0);
	indexCounts.append(1); indices.append(16); indexTargets.append(0);

	
	// Finally do the deformation
	MMatrix minv = m.inverse();
	unsigned i = 0, ptr = 0;
	unsigned running=0; // running count
	for (; !iter.isDone(); iter.next(), ++i) {
		int ic = indexCounts[i];
		if (ic == 0) continue;
		running += ic;
		MPoint tar;
		for (; (ptr < running) && (ptr < indices.length()); ++ptr) {
			int itar = indexTargets[ptr];
			MPointArray& pst = pointStack[itar];
			if (pst.length() == 0) {
				ic -= 1;
				continue;
			}
			MPoint& pvl = pst[indices[ptr]];
			tar += (worldStack[itar]) ? pvl * minv : pvl;
		}
		if (ic < 1) continue;

		tar = (MVector)tar / ic;
		MPoint pt = iter.position();
        float w = weightValue(data, multiIndex, iter.index()) * env;
		pt += (tar - pt) * w;
        iter.setPosition(pt);
		if (ptr >= indices.length()) break;
	}

    return stat;
}

