#pragma once
#include "DetourNavMesh.h"

enum BoundaryType
{
	INNER = 0,
	OUTER = 1,
	UNCONNECTED_EDGE = 3,
	TILE = 4
};

/// Contains all the data that should be filled for the Debug drawing.
/// Should be in sync with the C# side.
struct NavMeshDebugDrawData
{
	float* polyTrianglesPositions;
	unsigned char* polyTrianglesArea;
	int* polyTrianglesCount;
	int polyTrianglesMaxCapacity;

	float* boundariesPositions;
	unsigned char* boundariesType;
	int* boundariesLinesCount;
	int boundariesMaxCapacity;

	bool addPolyTriangle(const float** positions, unsigned char area)
	{
		if (*polyTrianglesCount == polyTrianglesMaxCapacity - 1)
		{
			// Can't add anymore triangles.
			return false;
		}
		
		int offset = *polyTrianglesCount * 9;
		for (int i = 0; i < 3; ++i)
		{
			polyTrianglesPositions[offset + i * 3] = positions[i][0];
			polyTrianglesPositions[offset + i * 3 + 1] = positions[i][1];
			polyTrianglesPositions[offset + i * 3 + 2] = positions[i][2];
		}

		polyTrianglesArea[*polyTrianglesCount] = area;
		
		(*polyTrianglesCount)++;
		
		return true;
	}

	bool addBoundary(const float** positions, BoundaryType boundaryType)
	{
		if (*boundariesLinesCount == boundariesMaxCapacity - 1)
		{
			// Can't add anymore triangles.
			return false;
		}

		int offset = *boundariesLinesCount * 6;
		for (int i = 0; i < 2; ++i)
		{
			boundariesPositions[offset + i * 3] = positions[i][0];
			boundariesPositions[offset + i * 3 + 1] = positions[i][1];
			boundariesPositions[offset + i * 3 + 2] = positions[i][2];
		}

		boundariesType[*boundariesLinesCount] = boundaryType;
		
		(*boundariesLinesCount)++;
		
		return true;
	}
};

/// Contains all the methods that are linked ot the debug drawing of the Tile NavMesh.
/// All the methods are heavily inspired by the DetourDebugDraw methods.
class NavMeshDebugDrawUtility
{
public:
	static bool fetchTileNavMeshDebugDrawData(const dtNavMesh& mesh, NavMeshDebugDrawData* debugDrawData); 

private:
	static bool fetchMeshTileData(const dtNavMesh& mesh, const dtMeshTile* tile, NavMeshDebugDrawData* debugDrawData);

	static bool fetchMeshBoundariesData(const dtMeshTile* tile, NavMeshDebugDrawData* debugDrawData);
	
	static float distancePtLine2d(const float* pt, const float* p, const float* q);
};
