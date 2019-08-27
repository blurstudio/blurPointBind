
from maya import OpenMaya as om, cmds
from dcc.maya.mayaToNumpy import mayaToNumpy, numpyToMaya
from scipy.spatial import cKDTree as KDTree
import numpy as np
from bisect import bisect_right

def getMeshPoints(shapeName, worldSpace=False):
	''' Get the points from the given mesh
	
	Arguments:
		shapeName (str): ShapeNode name
		worldSpace (bool): Whether to get the points in WorldSpace

	Returns:
		np.array: The list of points as a numpy array
	'''
	sl = om.MSelectionList()
	sl.add(shapeName)
	mesh = om.MDagPath()
	sl.getDagPath(0, mesh)
	meshFn = om.MFnMesh(mesh)
	mpa = om.MPointArray()
	space = om.MSpace.kWorld if worldSpace else om.MSpace.kObject
	meshFn.getPoints(mpa, space)
	return mayaToNumpy(mpa)

def getAllPoints(meshShapes, worldSpaces):
	''' Get all the points from the given meshes
	
	Arguments:
		meshShapes ([str, ...]): List of mesh ShapeNode names
		worldSpaces ([bool, ...]): Whether to get the points in WorldSpace

	Returns:
		np.array: The list of points as a numpy array
		[int, ...]: The number of points per mesh
	'''
	ptss = [getMeshPoints(ms, worldSpace=ws) for ms, ws in zip(meshShapes, worldSpaces)]
	return np.concatenate(ptss), [len(i) for i in ptss]

def getClosest(queryPoints, staticPoints, count=3, tol=0.01):
	''' Get the closest points to each query point within a tolerance

	Arguments:
		queryPoints (np.array): The points we will be finding the closest to
		staticPoints (np.array): The points we will be searching
		count (int): The number of closest points to return
		tol (float): The radius of the search for the closest points

	Returns:
		np.array: A (len(staticPoints)*count) array of corresponding points per query
			A value of len(staticPoints) means the match is invalid
	'''
	tree = KDTree(staticPoints)
	cpoints = tree.query(queryPoints, k=count, distance_upper_bound=tol)
	return cpoints[1]

def buildPointBindData(deformerMeshes, deformerWSs, boundMesh, boundWS, tolerance=0.01):
	''' Build the data that will be set on the blurPointBind input plugs

	Arguments:
		deformerMeshes ([str, ...]): List of mesh ShapeNode names that will deform the bound mesh
		deformerWSs ([bool, ...]): List of WorldSpace values per deformerMesh
		boundMesh (str): Name of the ShapeNode that will be deformed
		boundWS (bool): WorldSpace value for the bound mesh
		tolerance (float): The radius of the search for the closest points

	Returns:
		counts (np.array): The number of vertices to bind to per boundMesh vertex
		targets (np.array): For each bound vertex, the index of the deformerMesh
		idxs (np.array): For each bound vertex, the index of the vertex to bind to
	'''
	deformerPoints, pointCounts = getAllPoints(deformerMeshes, worldSpaces=deformerWSs)
	boundPoints = getMeshPoints(boundMesh, worldSpace=boundWS)
	nClosest = getClosest(boundPoints, deformerPoints, count=len(deformerMeshes), tol=tolerance)
	runningCounts = np.cumsum(pointCounts).tolist()
	runningCounts.insert(0, 0) # So I can subtract the runningCount from the vertex index directly

	if len(nClosest.shape) == 1:
		# Make sure nClosest is a 2d array
		nClosest = nClosest[..., None]

	invalidIdx = len(deformerPoints)
	targets, idxs, counts = [], [], []
	lastNonzero = None
	for nc in nClosest:
		count = 0
		for v in nc:
			if v == invalidIdx: continue
			# Find the insert index for v in runningCounts
			meshIdx = bisect_right(runningCounts, v)
			count += 1
			targets.append(meshIdx-1) # have to subtract one because of the insert(0, 0)
			idxs.append(v - runningCounts[meshIdx-1])
		counts.append(count)
		if count > 0:
			# This allows the deformer to bail early
			# if there aren't any more indices to compute
			lastNonzero = len(counts)
	counts = counts[:lastNonzero]
	return np.array(counts), np.array(targets), np.array(idxs)

def setPlugData(plugName, data):
	sl = om.MSelectionList()
	sl.add(plugName)
	plug = om.MPlug()
	sl.getPlug(0, plug)

	mfn = om.MFnIntArrayData()
	mObj = mfn.create()
	mfn.set(numpyToMaya(data, om.MIntArray))
	plug.setMObject(mObj)


def setPointBindData(deformer, counts, targets, idxs, meshes, worldSpaces, dfmIdx=0):
	''' Set the data on the blurPointBind deformer node

	Arguments:
		deformer (str): The name of the deformer node
		counts (np.array): The number of points in the list to bind to
		targets (np.array): The index of the mesh to bind to per-index
		idxs (np.array): The vertex index to bind to
		meshes ([str, ...]): The meshes to connect to the deformer
		dfmIdx (int): The mesh index of the deformed object. Defaults to 0
		worldSpace (bool): Whether to bind in worldSpace
	'''
	_grp = '{0}.targetGroup[{1}]'.format(deformer, dfmIdx)

	# Set the data on the plugs
	setPlugData('{0}.indexTargets'.format(_grp), targets)
	setPlugData('{0}.indexCounts'.format(_grp), counts)
	setPlugData('{0}.indexes'.format(_grp), idxs)

	# Connect the meshes and set the worldspace values
	for meshIdx, (mesh, ws) in enumerate(zip(meshes, worldSpaces)):
		meshOut = '{0}.outMesh'.format(mesh)
		meshPlug = '{0}.targets[{1}].targetMesh'.format(_grp, meshIdx)
		wsPlug = '{0}.targets[{1}].targetWorld'.format(_grp, meshIdx)
		cmds.connectAttr(meshOut, meshPlug, force=1)
		cmds.setAttr(wsPlug, ws)

def buildPointBind(mesh, controllers, worldSpace=False, tolerance=0.01):
	''' Build and connect the blurPointBind deformer

	Arguments:
		mesh (str): The mesh to add the deformer to
		controllers ([str, ...]): A list of controlling meshes
		worldSpace (bool): Whether to run the bind in worldSpace. Defaults to False
		tolerance (float): The tolerance of the closestPoint binding
	'''
	ctrlMeshes = []
	for ctrl in controllers:
		nType = cmds.nodeType(ctrl)
		if nType == 'transform':
			cShapes = cmds.listRelatives(ctrl, shapes=True, noIntermediate=True)
			if not cShapes:
				raise ValueError("A passed transform controller does not have any non-intermediate shapes: {0}".format(ctrl))
			elif len(cShapes) > 1:
				raise ValueError("A passed transform controller has multiple non-intermediate shapes: {0}".format(ctrl))
			ctrlMeshes.extend(cShapes)
		elif nType == 'mesh':
			ctrlMeshes.append(ctrl)
		else:
			raise ValueError("Expected a transform or mesh, got {0}: {1}".format(nType, ctrl))

	if not cmds.pluginInfo("blurPointBind", query=True, loaded=True):
		cmds.loadPlugin("blurPointBind")

	deformer = cmds.deformer(mesh, type="blurPointBind")[0]
	deformerWSs = [worldSpace] * len(ctrlMeshes)
	counts, targets, idxs = buildPointBindData(ctrlMeshes, deformerWSs, mesh, worldSpace, tolerance=tolerance)
	setPointBindData(deformer, counts, targets, idxs, ctrlMeshes, deformerWSs, dfmIdx=0)

from reloadPlugin import reloadPlugin, RELEASE_TYPES
base = r'D:\Users\tyler\Documents\GitHub\Plugins\blurPointBind'
plugName = 'blurPointBind'
openFile = r'D:\Users\tyler\Documents\GitHub\Plugins\blurPointBind\Useful\pbTest4.ma'
reloadPlugin(base, plugName, "RelWithDebInfo", openFile=openFile)
buildPointBind('Body', ['Head'], worldSpace=False, tolerance=0.01)




