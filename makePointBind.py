from maya import OpenMaya as om, cmds
from dcc.maya.mayaToNumpy import mayaToNumpy, numpyToMaya
from scipy.spatial import cKDTree as KDTree
import numpy as np
from bisect import bisect_left


def buildMergedMObject(meshShapes, worldSpace=False):
	''' Merge the given meshes into a single MObject
	Probably not used as part of the point bind, but makes it easy to
	build one octree on all the meshes you're binding to
	
	Arguments:
		meshShapes ([str, ...]): List of mesh ShapeNode names
		worldSpace (bool): Whether to merge in worldSpace

	Returns:
		MObject : The merged MObject
		MFnMesh : The MeshFn attached to the MObject
	'''


	pointses, faceses, countses = [], [], []

	for shapeName in meshShapes:
		sl = om.MSelectionList()
		sl.add(shapeName)
		mesh = om.MDagPath()
		sl.getDagPath(0, mesh)
		meshFn = om.MFnMesh(mesh)
		mpa = om.MPointArray()
		space = om.MSpace.kWorld if worldSpace else om.MSpace.kObject
		meshFn.getPoints(mpa, space)

		counts = []
		faces = []
		vIdx = om.MIntArray()
		for i in range(meshFn.numPolygons()):
			meshFn.getPolygonVertices(i, vIdx)
			counts.append(vIdx.length())
			faces.append(mayaToNumpy(vIdx))

		pointses.append(mayaToNumpy(mpa))
		countses.append(np.array(counts))
		faceses.append(np.concatenate(faces))

	pointses = numpyToMaya(np.concatenate(pointses), om.MPointArray)
	faceses = numpyToMaya(np.concatenate(faceses), om.MIntArray)
	countses = numpyToMaya(np.concatenate(countses), om.MIntArray)

	mergedFn = om.MFnMesh()
	merged = mergedFn.create(len(pointses), len(countses), pointses, countses, faceses)

	return merged, mergedFn

def projectClosestPoints(meshShapeName, ontoShapeName, worldSpace=False, tolerance=0.01):
	''' Get the closest points from one mesh on another, and get a simple approximation
	This is a nice piece of code, but we won't use it for the point bind

	Args:
		meshShapeName (str): The mesh shape node whose points get looped over.
		ontoShapeName (str): The mesh shape node that we find the closest point on
		worldSpace (bool): Whether to do this in worldspace
		tolerance (float): A tolerance for the barycentric approximation
	
	Returns:
		[[MPoint, ...], ...]: The closest (up to 3) MPoints per vertex
	
	'''
	sl = om.MSelectionList()
	sl.add(meshShapeName)
	mesh = om.MDagPath()
	sl.getDagPath(0, mesh)
	meshFn = om.MFnMesh(mesh)
	mpa = om.MPointArray()
	space = om.MSpace.kWorld if worldSpace else om.MSpace.kObject
	meshFn.getPoints(mpa, space)


	sl = om.MSelectionList()
	sl.add(ontoShapeName)
	onto = om.MDagPath()
	sl.getDagPath(0, onto)
	wmat = onto.inclusiveMatrix() if worldSpace else om.MMatrix()

	targetWMatInv = wmat.inverse()
	octree = om.MMeshIntersector()
	octree.create(onto.node())
	ontoFn = om.MFnMesh(onto)


	# Build the return-by-reference script utils
	uUtil = om.MScriptUtil()
	uUtil.createFromDouble(0.0)
	uPtr = uUtil.asFloatPtr()
	vUtil = om.MScriptUtil()
	vUtil.createFromDouble(0.0)
	vPtr = vUtil.asFloatPtr()
	triUtil = om.MScriptUtil()
	triUtil.createFromList([0, 0, 0], 3)
	triPtr = triUtil.asIntPtr()

	out = []
	for idx in xrange(mpa.length()):
		tpt = mpa[idx] * targetWMatInv
		res = om.MPointOnMesh() # result
		octree.getClosestPoint(tpt, res, 10.0)

		res.getBarycentricCoords(uPtr, vPtr)

		uvw = [uUtil.getFloat(uPtr), vUtil.getFloat(vPtr), 0.0]
		uvw[2] = 1.0 - sum(uvw)

		faceIdx = res.faceIndex()
		triIdx = res.triangleIndex()
		ontoFn.getPolygonTriangleVertices(faceIdx, triIdx, triPtr)
		triVerts = [triUtil.getIntArrayItem(triPtr, i) for i in range(3)]

		maxVal = max(uvw)
		minVal = min(uvw)
		if (1.0 - maxVal) < tolerance:
			maxIdx = uvw.index(maxVal)
			triVerts = [triVerts[maxIdx]]
		elif minVal < tolerance:
			minIdx = uvw.index(minVal)
			triVerts.pop(minIdx)
		out.append(triVerts)

	return out







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
		meshes (np.array): For each bound vertex, the index of the deformerMesh
	'''
	deformerPoints, pointCounts = getAllPoints(deformerMeshes, worldSpaces=deformerWSs)
	boundPoints = getMeshPoints(boundMesh, worldSpace=boundWS)
	nClosest = getClosest(boundPoints, deformerPoints, count=len(deformerMeshes), tol=tolerance)
	runningCounts = np.cumsum(pointCounts).tolist()
	runningCounts.insert(0, 0) # So I can subtract the runningCount from the vertex index directly

	# loop through the nClosest
	# check for values that == len(deformerPoints)
	# check for points between values in the running sum of pointCounts
	# translate the data properly into chunks
	# turn into maya values and return

	invalidIdx = len(deformerPoints)
	meshes, idxs, counts = [], [], []
	for nc in nClosest:
		count = 0
		for v in nc:
			if v == invalidIdx: continue
			# Find the insert index for v in runningCounts
			meshIdx = bisect_left(runningCounts, v)
			count += 1
			meshes.append(meshIdx-1) # have to subtract one because of the insert(0, 0)
			idxs.append(v - runningCounts[meshIdx])
		counts.append(count)
	return np.array(counts), np.array(meshes), np.array(idxs)



def _getPlug(plugName):
	sl = om.MSelectionList()
	sl.add(plugName)
	plug = om.MPlug()
	sl.getPlug(0, plug)
	return plug.asMObject()

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

	# Build the plug names
	iTarPlug = _getPlug('{0}.indexTargets'.format(_grp))
	iCountPlug = _getPlug('{0}.indexCounts'.format(_grp))
	indexPlug = _getPlug('{0}.indexes'.format(_grp))

	# Set the data on the plugs
	om.MFnIntArrayData(iTarPlug).set(numpyToMaya(targets, om.MIntArray))
	om.MFnIntArrayData(iCountPlug).set(numpyToMaya(counts, om.MIntArray))
	om.MFnIntArrayData(indexPlug).set(numpyToMaya(idxs, om.MIntArray))

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

	deformer = cmds.deformer(mesh, type="blurPointBind")
	deformerWSs = [worldSpace] * len(ctrlMeshes)
	counts, targets, idxs = buildPointBindData(ctrlMeshes, deformerWSs, mesh, worldSpace, tolerance=tolerance)
	setPointBindData(deformer, counts, targets, idxs, ctrlMeshes, deformerWSs, dfmIdx=0)

