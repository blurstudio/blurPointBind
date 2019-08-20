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

#define DEFORMER_NAME "blurPointBind"

class PointBindDeformer : public MPxDeformerNode {
public:
	PointBindDeformer() {};
	virtual ~PointBindDeformer() {};

    static void* creator();
    static MStatus initialize();
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
    static MTypeId id;

	static MObject aTargetGroup;
	static MObject aIndices;
	static MObject aIndexTargets;
	static MObject aIndexCounts;
	static MObject aTargets;
};

