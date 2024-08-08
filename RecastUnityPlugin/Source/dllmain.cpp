#include <windows.h>
#include <stdint.h>

#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "RecastUnityPluginManager.h"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
	
    return TRUE;
}

#define DllExport __declspec(dllexport)

extern "C"
{
	const int PATH_MAX_CAPACITY = 256;
	
	DllExport bool Initialize()
	{
		return RecastUnityPluginManager::initialize();
	}

	DllExport void Dispose()
	{
		RecastUnityPluginManager::dispose();
	}
	
	// Allocate Navmesh 
	DllExport dtStatus CreateNavMesh(const void* config, const float* bmin, const float* bmax, const void* inputGeometry, void*& allocatedNavMesh)
	{
		return RecastUnityPluginManager::createNavMesh(*((const NavMeshBuildConfig*)config), bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry), allocatedNavMesh);
	}

	// Creates a TileNavMesh from some global mesh data.
	DllExport dtStatus CreateTileNavMesh(const void* config, float tileSize, bool buildAllTiles,
		const float* bmin, const float* bmax,
		const void* inputGeometry, void*& allocatedNavMesh, int* tilesCount)
	{
		return RecastUnityPluginManager::createTileNavMesh(*((const NavMeshBuildConfig*)config), tileSize, buildAllTiles, bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry), allocatedNavMesh, tilesCount);
	}

	DllExport void AddTile(int* tilesCoordinates, const void* config, float tileSize,
		const float* bmin, const float* bmax,
		const void* inputGeometry, void*& allocatedNavMesh, bool dontRecomputeBounds = false)
	{
		return RecastUnityPluginManager::addTile(tilesCoordinates, *((const NavMeshBuildConfig*)config), tileSize, bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry), (dtNavMesh*)allocatedNavMesh, dontRecomputeBounds);
	}
	
	// Dispose Navmesh
	DllExport void DisposeNavMesh(void*& allocatedNavMesh)
	{
		if (allocatedNavMesh != nullptr)
		{
			RecastUnityPluginManager::disposeNavMesh(allocatedNavMesh);
		}
	}
	
	// Create Navmesh query (from a specific navmesh)
	DllExport dtStatus CreateNavMeshQuery(const void* navMesh, int maxNodes, void*& allocatedNavMeshQuery)
	{
		return RecastUnityPluginManager::createNavMeshQuery(navMesh, maxNodes, allocatedNavMeshQuery);
	}
	
	// Dispose Navmesh query
	DllExport void DisposeNavMeshQuery(void*& allocatedNavMeshQuery)
	{
		if (allocatedNavMeshQuery != nullptr)
		{
			RecastUnityPluginManager::disposeNavMeshQuery(allocatedNavMeshQuery);
		}
	}

	// Path query. Calls FindPath then FindStraightPath.
	// Parameters
	// - [in] navmesh query (references the navmesh)
	// - [in] start + end positions (used to find the nearest poly)
	// - [in] polygon search extents for start + end
	// - [in] query filter ? Probably later
	// - [out] positions found on the nearest polygons for start + end?
	// - [out] path (float* array) containing the positions (3 floats per path node) of the path
	// - [out] path positions count
	// - [returns] dtStatus (unsigned int)
	DllExport dtStatus FindStraightPath(void* navMeshQuery, const float* startPosition, const float* endPosition, const float* polygonSearchExtents, float* pathPositions,
		int* pathPositionsCount, int pathMaxSize = PATH_MAX_CAPACITY)
	{
		// Copied from NavMeshTesterTool.cpp
		auto query = (dtNavMeshQuery*)navMeshQuery;
		// TEMP: For now we don't use any specific filter.
		dtQueryFilter filter;
		dtPolyRef startPolyRef, endPolyRef;
		query->findNearestPoly(startPosition, polygonSearchExtents, &filter, &startPolyRef, 0);
		query->findNearestPoly(endPosition, polygonSearchExtents, &filter, &endPolyRef, 0);
		
		pathMaxSize =  PATH_MAX_CAPACITY < pathMaxSize ? PATH_MAX_CAPACITY : pathMaxSize;
		dtPolyRef pathPolys[PATH_MAX_CAPACITY];
		int pathCount;
		query->findPath(startPolyRef, endPolyRef, startPosition, endPosition, &filter, pathPolys, &pathCount, pathMaxSize);
		if (pathCount > 0)
		{
			// In case of partial path, make sure the end point is clamped to the last polygon.
			float epos[3];
			dtVcopy(epos, endPosition);
			if (pathPolys[pathCount-1] != endPolyRef)
				query->closestPointOnPoly(pathPolys[pathCount-1], endPosition, epos, 0);
			
			dtPolyRef straightPathPolys[PATH_MAX_CAPACITY];
			unsigned char straightPathFlags[PATH_MAX_CAPACITY];
			
			// TEMP: no straight path options for now.
			dtStatus pathQueryResult = query->findStraightPath(startPosition, epos, pathPolys, pathCount,
										 pathPositions, straightPathFlags,
										 straightPathPolys, pathPositionsCount, pathMaxSize, 0);
			return pathQueryResult;
		}

		return DT_FAILURE;
	}

	
	// Debug
	DllExport int GetNavMeshTrianglesCount(const void* navMeshHandle)
	{
		const dtNavMesh* navMesh = (const dtNavMesh*)navMeshHandle;
		int trianglesCount = 0;
		for (int i = 0; i < navMesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navMesh->getTile(i);
			if (!tile->header) continue;
			for (int i = 0; i < tile->header->polyCount; ++i)
			{
				const dtPoly* p = &tile->polys[i];
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
					continue;
			
				const dtPolyDetail* pd = &tile->detailMeshes[i];
				trianglesCount += pd->triCount;
			}
		}

		return trianglesCount;
	}
	
	DllExport void GetNavMeshTriangles(float* trianglePositions, const void* navMeshHandle)
	{
		const dtNavMesh* navMesh = (const dtNavMesh*)navMeshHandle;
		int triangleVertexPositionIndex = 0;
		for (int i = 0; i < navMesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navMesh->getTile(i);
			if (!tile->header) continue;
			
			for (int i = 0; i < tile->header->polyCount; ++i)
			{
				const dtPoly* p = &tile->polys[i];
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
					continue;
		
				const dtPolyDetail* pd = &tile->detailMeshes[i];
				
				for (int j = 0; j < pd->triCount; ++j)
				{
					const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
					for (int k = 0; k < 3; ++k)
					{
						float* pos;
						if (t[k] < p->vertCount)
							pos = &tile->verts[p->verts[t[k]]*3];
						else
							pos = &tile->detailVerts[(pd->vertBase+t[k]-p->vertCount)*3];
						
						trianglePositions[triangleVertexPositionIndex] = pos[0];
						trianglePositions[triangleVertexPositionIndex + 1] = pos[1];
						trianglePositions[triangleVertexPositionIndex + 2] = pos[2];
						triangleVertexPositionIndex += 3;
					}
				}
			}
		}
	}
}