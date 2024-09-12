#include <windows.h>
#include <stdint.h>

#include "BlockArea.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "NavMeshDebugDrawUtility.h"
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

	// Creates an empty TileNavMesh
	DllExport dtStatus CreateTileNavMesh(const void* config, float tileSize,
	const float* bmin, const float* bmax, void*& allocatedNavMesh, int* tilesCount)
	{
		return RecastUnityPluginManager::createTileNavMesh(*((const NavMeshBuildConfig*)config), tileSize,
			bmin, bmax, allocatedNavMesh, tilesCount);
	}

	DllExport void AddTile(int* tilesCoordinates, const void* config, float tileSize,
	const float* bmin, const float* bmax,
	const void* inputGeometry, void*& allocatedNavMesh, const BlockArea* blockAreas, int blocksCount)
	{
		return RecastUnityPluginManager::addTile(tilesCoordinates, *((const NavMeshBuildConfig*)config), tileSize,
			bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry),
			(dtNavMesh*)allocatedNavMesh, blockAreas, blocksCount);
	}
	
	// Creates a TileNavMesh from some global mesh data.
	DllExport dtStatus CreateTileNavMeshWithChunkyMesh(const void* config, float tileSize, bool buildAllTiles,
		const float* bmin, const float* bmax,
		const void* inputGeometry, void*& allocatedNavMesh, void*&computedChunkyTriMesh, int* tilesCount)
	{
		return RecastUnityPluginManager::createTileNavMeshWithChunkyMesh(*((const NavMeshBuildConfig*)config), tileSize, buildAllTiles,
			bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry),
			allocatedNavMesh, computedChunkyTriMesh, tilesCount);
	}

	DllExport void AddTileWithChunkyMesh(int* tilesCoordinates, const void* config, float tileSize,
		const float* bmin, const float* bmax,
		const void* inputGeometry, void*& allocatedNavMesh, const void* chunkTriMesh, bool dontRecomputeBounds = false)
	{
		return RecastUnityPluginManager::addTileWithChunkyMesh(tilesCoordinates, *((const NavMeshBuildConfig*)config), tileSize,
			bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry),
			(dtNavMesh*)allocatedNavMesh, (const rcChunkyTriMesh*)chunkTriMesh, dontRecomputeBounds);
	}
	
	// Dispose Navmesh
	DllExport void DisposeNavMesh(void* allocatedNavMesh)
	{
		if (allocatedNavMesh != nullptr)
		{
			RecastUnityPluginManager::disposeNavMesh(allocatedNavMesh);
		}
	}

	DllExport void DisposeChunkyTriMesh(void* chunkyMesh)
	{
		if (chunkyMesh != nullptr)
		{
			rcChunkyTriMesh* chunkyTriMesh = (rcChunkyTriMesh*)chunkyMesh;
			delete chunkyTriMesh;
		}
	}
	
	// Create Navmesh query (from a specific navmesh)
	DllExport dtStatus CreateNavMeshQuery(const void* navMesh, int maxNodes, void*& allocatedNavMeshQuery)
	{
		return RecastUnityPluginManager::createNavMeshQuery(navMesh, maxNodes, allocatedNavMeshQuery);
	}
	
	// Dispose Navmesh query
	DllExport void DisposeNavMeshQuery(void* allocatedNavMeshQuery)
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
	// - [in] query filter
	// - [out] positions found on the nearest polygons for start + end?
	// - [out] path (float* array) containing the positions (3 floats per path node) of the path
	// - [out] path positions count
	// - [returns] dtStatus (unsigned int)
	DllExport dtStatus FindStraightPath(void* navMeshQuery, const float* startPosition, const float* endPosition, const float* polygonSearchExtents,
		const dtQueryFilter* filter, float* pathPositions,
		int* pathPositionsCount, int pathMaxSize = PATH_MAX_CAPACITY)
	{
		// Copied from NavMeshTesterTool.cpp
		auto query = (dtNavMeshQuery*)navMeshQuery;
		// TEMP: For now we don't use any specific filter.
		dtPolyRef startPolyRef, endPolyRef;
		query->findNearestPoly(startPosition, polygonSearchExtents, filter, &startPolyRef, 0);
		query->findNearestPoly(endPosition, polygonSearchExtents, filter, &endPolyRef, 0);
		
		pathMaxSize =  PATH_MAX_CAPACITY < pathMaxSize ? PATH_MAX_CAPACITY : pathMaxSize;
		dtPolyRef pathPolys[PATH_MAX_CAPACITY];
		int foundPathSize;
		dtStatus pathResult = query->findPath(startPolyRef, endPolyRef, startPosition, endPosition, filter, pathPolys, &foundPathSize, pathMaxSize);
		if (foundPathSize > 0)
		{
			// In case of partial path, make sure the end point is clamped to the last polygon.
			float epos[3];
			dtVcopy(epos, endPosition);
			if (pathPolys[foundPathSize-1] != endPolyRef)
				query->closestPointOnPoly(pathPolys[foundPathSize-1], endPosition, epos, 0);
			
			dtPolyRef straightPathPolys[PATH_MAX_CAPACITY];
			unsigned char straightPathFlags[PATH_MAX_CAPACITY];
			
			// TEMP: no straight path options for now.
			dtStatus pathQueryResult = query->findStraightPath(startPosition, epos, pathPolys, foundPathSize,
										 pathPositions, straightPathFlags,
										 straightPathPolys, pathPositionsCount, pathMaxSize, 0);

			// Pass some status info from the first path search to the final status.
			if ((pathResult & DT_PARTIAL_RESULT) != 0)
			{
				pathQueryResult |= DT_PARTIAL_RESULT;
			}

			if ((pathResult & DT_OUT_OF_NODES) != 0)
			{
				pathQueryResult |= DT_OUT_OF_NODES;
			}
			
			return pathQueryResult;
		}

		return DT_FAILURE;
	}
	
	// Debug
	DllExport int GetPolyTrianglesCount(const void* navMeshHandle)
	{
		const dtNavMesh* navMesh = (const dtNavMesh*)navMeshHandle;
		int polyTrianglesCount = 0;
		for (int i = 0; i < navMesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navMesh->getTile(i);
			if (!tile->header) continue;
			for (int k = 0; k < tile->header->polyCount; ++k)
			{
				const dtPoly* p = &tile->polys[k];
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
					continue;
			
				const dtPolyDetail* pd = &tile->detailMeshes[k];
				polyTrianglesCount += pd->triCount;
			}
		}
		return polyTrianglesCount;
	}

	DllExport bool FetchNavMeshDebugDrawData(const void* navMeshHandle, void* data)
	{
		const dtNavMesh* navMesh = (const dtNavMesh*)navMeshHandle;
		NavMeshDebugDrawData* debugDrawData = (NavMeshDebugDrawData*)data;
		return NavMeshDebugDrawUtility::fetchTileNavMeshDebugDrawData(*navMesh, debugDrawData);
	}
}