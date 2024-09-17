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
	/// The positions of all the vertices from the polygon triangles (3 items per vertex).
	float* polyTrianglesPositions;
	/// The area of all the polygon triangles.
	unsigned char* polyTrianglesArea;
	/// The number of polygon triangles.
	int* polyTrianglesCount;
	/// The max number of polygon triangles that the data can gather.
	int polyTrianglesMaxCapacity;

	/// The boundaries positions (3 items per position). The positions are organised line by line (2 positions per line)
	float* boundariesPositions;
	/// The type of boundary for each line.
	unsigned char* boundariesType;
	/// The number of lines that were gathered.
	int* boundariesLinesCount;
	/// The max number of boundary lines that the data can gather.
	int boundariesMaxCapacity;

	bool addPolyTriangle(const float** positions, const unsigned char area)
	{
		if (*polyTrianglesCount == polyTrianglesMaxCapacity - 1)
		{
			// Can't add anymore triangles.
			return false;
		}

		const int offset = *polyTrianglesCount * 9;
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

	bool addBoundary(const float** positions, const BoundaryType boundaryType)
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
