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

def getAllPoints(meshShapes, worldSpace=False):
	''' Get all the points from the given meshes
	
	Arguments:
		meshShapes ([str, ...]): List of mesh ShapeNode names
		worldSpace (bool): Whether to get the points in WorldSpace

	Returns:
		np.array: The list of points as a numpy array
		[int, ...]: The number of points per mesh
	'''
	ptss = [getMeshPoints(ms, worldSpace=worldSpace) for ms in meshShapes]
	return np.concatenate(ptss), [len(i) for i in ptss]

def getClosest(queryPoints, staticPoints, count=3, tol=0.01):
	''' Get the closest points to each query point within a tolerance

	Arguments:
		queryPoints (np.array): The points we will be finding the closest to
		staticPoints (np.array): The points we will be searching
		count (int): The number of closest points to return
		tol (float): The radius of the search for the closest points

	Returns:
		np.array: A (len(staticPoints)*count) array of corresponding points per
			query. A value of len(staticPoints) means the match is invalid.
	'''

	tree = KDTree(staticPoints)
	cpoints = tree.query(queryPoints, k=count, distance_upper_bound=tol)
	return cpoints[1]

def buildPointBindData(deformerMeshes, boundMesh, tolerance=0.01, worldSpace=False):
	''' Build the blurPointBind, fill in the data, and connect

	Arguments:
		meshShapes ([str, ...]): List of mesh ShapeNode names that will deform the bound mesh
		boundMesh (str): Name of the ShapeNode that will be deformed
		tolerance (float): The radius of the search for the closest points
		worldSpace (bool): Whether to get the points in WorldSpace

	Returns:
		counts (np.array): The number of vertices to bind to per boundMesh vertex
		meshes (np.array): For each bound vertex, the index of the deformerMesh
		


	'''

	deformerPoints, pointCounts = getAllPoints(deformerMeshes, worldSpace=worldSpace)
	boundPoints = getMeshPoints(boundMesh, worldSpace=worldSpace)
	nClosest = getClosest(boundPoints, deformerPoints, count=len(deformerMeshes), tol=tolerance)
	runningCounts = np.cumsum(pointCounts)

	# loop through the nClosest
	# check for values that == len(deformerPoints)
	# check for points between values in the running sum of pointCounts
	# translate the data properly into chunks
	# turn into maya values and return


	meshes, idxs, counts = [], [], []
	for nc in nClosest:
		count = 0
		for v in nc:
			if v == len(deformerPoints): continue
			meshIdx = bisect_left(runningCounts, v)
			count += 1
			meshes.append(meshIdx)
			idxs.append(v)
		counts.append(count)
	return np.array(counts), np.array(meshes), np.array(idxs)




