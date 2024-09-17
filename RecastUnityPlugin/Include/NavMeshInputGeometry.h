#pragma once

/// Contains all the geometry used to compute the NavMesh.
struct NavMeshInputGeometry
{
	float* vertices;
	int* triangles;
	int verticesCount;
	int trianglesCount;
};
