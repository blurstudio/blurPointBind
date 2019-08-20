#include "blurPointBind.h"
#ifdef GPU
	#include "blurPointBindGPU.h"
#endif
#include <maya/MFnPlugin.h>

// standard initialization procedures
MStatus initializePlugin(MObject obj) {
    MStatus result;
    MFnPlugin plugin(obj, "Blur Studio", "1.0", "Any");
    result = plugin.registerNode(DEFORMER_NAME, PointBindDeformer::id, PointBindDeformer::creator,
                                  PointBindDeformer::initialize, MPxNode::kDeformerNode);

#ifdef GPU
    MString nodeClassName(DEFORMER_NAME);
    MString registrantId("BlurPlugin");
    MGPUDeformerRegistry::registerGPUDeformerCreator(
        nodeClassName,
        registrantId,
        PointBindGPUDeformer::getGPUDeformerInfo());

    MGPUDeformerRegistry::addConditionalAttribute(
        nodeClassName,
        registrantId,
        MPxDeformerNode::envelope);
    PointBindGPUDeformer::pluginLoadPath = plugin.loadPath();
#endif

    return result;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus result;
    MFnPlugin plugin(obj);
    result = plugin.deregisterNode(PointBindDeformer::id);

#ifdef GPU
    MString nodeClassName(DEFORMER_NAME);
    MString registrantId("BlurPlugin");
    MGPUDeformerRegistry::deregisterGPUDeformerCreator(nodeClassName, registrantId);
#endif

    return result;
}

