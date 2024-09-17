#include <windows.h>
#include <stdint.h>

#include "BlockArea.h"
#include "BuildContext.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "NavMeshDebugDrawUtility.h"
#include "RecastUnityPluginManager.h"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
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

	DllExport void Dispose(int environmentId)
	{
		RecastUnityPluginManager::dispose(environmentId);
	}

	// Allocate Navmesh

	/// Creates a non-tile NavMesh.
	DllExport dtStatus CreateNavMesh(const void* config, const float* bmin, const float* bmax, const void* inputGeometry,
	                                 void*& allocatedNavMesh, int environmentId)
	{
		return RecastUnityPluginManager::createNavMesh(*((const NavMeshBuildConfig*)config), bmin, bmax,
		                                               *((const NavMeshInputGeometry*)inputGeometry), allocatedNavMesh,
		                                               environmentId);
	}

	/**
	* \brief Creates an empty TileNavMesh. Tiles should be built later.
	* \param config Contains all the parameters that should be used to build the NavMesh.
	* \param tileSize The size of the tile (m)
	* \param bmin The min bounds (world coordinates) of the whole navMesh
	* \param bmax The max bounds (world coordinates) of the whole navMesh
	* \param allocatedNavMesh The returned allocated NavMesh.
	* \param tilesCount The returned tiles number
	* \param environmentId The environment id the NavMesh is linked to.
	* \return DT_SUCCESS if success, DT_FAILURE and some other flags if it failed.
	*/
	DllExport dtStatus CreateTileNavMesh(const void* config, float tileSize,
	                                     const float* bmin, const float* bmax, void*& allocatedNavMesh, int* tilesCount,
	                                     int environmentId)
	{
		return RecastUnityPluginManager::createTileNavMesh(*((const NavMeshBuildConfig*)config), tileSize,
		                                                   bmin, bmax, allocatedNavMesh, tilesCount, environmentId);
	}

	/**
	 * \brief Builds a tile for a tile NavMesh.
	 * \param tileCoordinates The coordinate of the tile.
	 * \param config Contains all the parameters that should be used to build the NavMesh.
	 * \param tileSize The size of the tile (m)
	 * \param bmin The min bounds (world coordinates) of the whole navMesh
	 * \param bmax The max bounds (world coordinates) of the whole navMesh
	 * \param inputGeometry The geometry (vertices + triangles) that should be used to generate the tile. Should contain
	 * the geometry located in the bounds + a little offset around the bounds to make sure the connection between the tiles is correct.
	 * \param navMesh The NavMesh we should add a tile on.
	 * \param blockAreas The block areas of the tile, to potentially flag some areas of the tile.
	 * \param blocksCount The number of block areas.
	 * \param contextData The data that the context must use. Used to return the timings to the C# side.
	 */
	DllExport void AddTile(int* tileCoordinates, const void* config, float tileSize,
	                       const float* bmin, const float* bmax,
	                       const void* inputGeometry, void*& navMesh, const BlockArea* blockAreas, int blocksCount,
	                       void* contextData)
	{
		TimeVal* timings = (TimeVal*)contextData;
		BuildContext buildContext(timings);
		return RecastUnityPluginManager::addTile(tileCoordinates, *((const NavMeshBuildConfig*)config), tileSize,
		                                         bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry),
		                                         (dtNavMesh*)navMesh, blockAreas, blocksCount, &buildContext);
	}

	/// Creates a TileNavMesh by building a ChunkyMesh. Not used anymore for now.
	DllExport dtStatus CreateTileNavMeshWithChunkyMesh(const void* config, float tileSize, bool buildAllTiles,
	                                                   const float* bmin, const float* bmax,
	                                                   const void* inputGeometry, void*& allocatedNavMesh,
	                                                   void*& computedChunkyTriMesh, int* tilesCount, int environmentId)
	{
		return RecastUnityPluginManager::createTileNavMeshWithChunkyMesh(*((const NavMeshBuildConfig*)config), tileSize,
		                                                                 buildAllTiles,
		                                                                 bmin, bmax,
		                                                                 *((const NavMeshInputGeometry*)inputGeometry),
		                                                                 allocatedNavMesh, computedChunkyTriMesh,
		                                                                 tilesCount, environmentId);
	}

	/// Adds a tile to a TileNavMesh by building a ChunkyMesh. Not used anymore for now.
	DllExport void AddTileWithChunkyMesh(int* tilesCoordinates, const void* config, float tileSize,
	                                     const float* bmin, const float* bmax,
	                                     const void* inputGeometry, void*& allocatedNavMesh, const void* chunkTriMesh,
	                                     bool dontRecomputeBounds = false)
	{
		return RecastUnityPluginManager::addTileWithChunkyMesh(tilesCoordinates, *((const NavMeshBuildConfig*)config),
		                                                       tileSize,
		                                                       bmin, bmax, *((const NavMeshInputGeometry*)inputGeometry),
		                                                       (dtNavMesh*)allocatedNavMesh,
		                                                       (const rcChunkyTriMesh*)chunkTriMesh, dontRecomputeBounds);
	}

	/// Dispose the NavMesh passed in parameter.
	DllExport void DisposeNavMesh(void* allocatedNavMesh, int environmentId)
	{
		if (allocatedNavMesh != nullptr)
		{
			RecastUnityPluginManager::disposeNavMesh(allocatedNavMesh, environmentId);
		}
	}

	/// Dispose the NavMeshQuery passed in parameter.
	DllExport void DisposeChunkyTriMesh(void* chunkyMesh)
	{
		if (chunkyMesh != nullptr)
		{
			rcChunkyTriMesh* chunkyTriMesh = (rcChunkyTriMesh*)chunkyMesh;
			delete chunkyTriMesh;
		}
	}

	/// Create a NavMesh query (from a specific navmesh)
	DllExport dtStatus CreateNavMeshQuery(const void* navMesh, int maxNodes, void*& allocatedNavMeshQuery,
	                                      int environmentId)
	{
		return RecastUnityPluginManager::createNavMeshQuery(navMesh, maxNodes, allocatedNavMeshQuery, environmentId);
	}

	// Dispose the NavMesh query passed in parameter.
	DllExport void DisposeNavMeshQuery(void* allocatedNavMeshQuery, int environmentId)
	{
		if (allocatedNavMeshQuery != nullptr)
		{
			RecastUnityPluginManager::disposeNavMeshQuery(allocatedNavMeshQuery, environmentId);
		}
	}

	/**
	 * \brief Computes a path. Calls FindPath then FindStraightPath.
	 * \param navMeshQuery The navmesh query to used (references the navmesh).
	 * \param startPosition The start position of the path.
	 * \param endPosition The end position of the path.
	 * \param polygonSearchExtents The search extents to use when trying to find a suitable polygon for the start and end positions.
	 * \param filter A filter for the path (for instance to exclude liquids)
	 * \param pathPositions The positions in the computed path.
	 * \param pathPositionsCount The number of positions in the path.
	 * \param pathMaxSize The max size of the path.
	 *\return DT_SUCCESS if success, DT_FAILURE and some other flags if it failed.
	 */
	DllExport dtStatus FindStraightPath(void* navMeshQuery, const float* startPosition, const float* endPosition,
	                                    const float* polygonSearchExtents,
	                                    const dtQueryFilter* filter, float* pathPositions,
	                                    int* pathPositionsCount, int pathMaxSize = PATH_MAX_CAPACITY)
	{
		// Copied from NavMeshTesterTool.cpp
		auto query = (dtNavMeshQuery*)navMeshQuery;
		dtPolyRef startPolyRef, endPolyRef;
		query->findNearestPoly(startPosition, polygonSearchExtents, filter, &startPolyRef, 0);
		query->findNearestPoly(endPosition, polygonSearchExtents, filter, &endPolyRef, 0);

		pathMaxSize = PATH_MAX_CAPACITY < pathMaxSize ? PATH_MAX_CAPACITY : pathMaxSize;
		dtPolyRef pathPolys[PATH_MAX_CAPACITY];
		int foundPathSize;
		dtStatus pathResult = query->findPath(startPolyRef, endPolyRef, startPosition, endPosition, filter, pathPolys,
		                                      &foundPathSize, pathMaxSize);
		if (foundPathSize > 0)
		{
			// In case of partial path, make sure the end point is clamped to the last polygon.
			float epos[3];
			dtVcopy(epos, endPosition);
			if (pathPolys[foundPathSize - 1] != endPolyRef)
				query->closestPointOnPoly(pathPolys[foundPathSize - 1], endPosition, epos, 0);

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
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
					continue;

				const dtPolyDetail* pd = &tile->detailMeshes[k];
				polyTrianglesCount += pd->triCount;
			}
		}
		return polyTrianglesCount;
	}

	/// Fetches all the data that is necessary to draw the NavMesh.
	DllExport bool FetchNavMeshDebugDrawData(const void* navMeshHandle, void* data)
	{
		const dtNavMesh* navMesh = (const dtNavMesh*)navMeshHandle;
		NavMeshDebugDrawData* debugDrawData = (NavMeshDebugDrawData*)data;
		return NavMeshDebugDrawUtility::fetchTileNavMeshDebugDrawData(*navMesh, debugDrawData);
	}
}
