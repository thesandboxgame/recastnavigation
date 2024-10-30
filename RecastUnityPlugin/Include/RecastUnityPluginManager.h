#ifndef RECAST_UNITY_PLUGIN_MANAGER_H
#define RECAST_UNITY_PLUGIN_MANAGER_H

#include <map>
#include "BlockArea.h"
#include "BuildContext.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "NavMeshBuildConfig.h"
#include "NavMeshInputGeometry.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"

// Copy of SamplePartitionType 
enum PartitionType
{
	PARTITION_WATERSHED,
	PARTITION_MONOTONE,
	PARTITION_LAYERS
};

// This is used to mark polygons when building it.
enum PolyAreas
{
	POLYAREA_DEFAULT = 0x01,
	POLYAREA_LIQUID = 0x02,
};

// This is used by the Nav Mesh queries to know if the agent has to ability to cross some polygons.
enum PolyFlags
{
	POLYFLAGS_WALK = 0x01,
	// Default area type. Can walk on it (ground, grass, etc.)
	POLYFLAGS_SWIM = 0x02,
	// Area type for liquids. Requires to swim (water, lava, etc.).
	SAMPLE_POLYFLAGS_ALL = 0xffff // All abilities.
};

/// A singleton that contains all the allocated data for the C# side.
/// Before using it, you should call RecastUnityPluginManager::Initialize()
class RecastUnityPluginManager
{
public:
	/// Creates the singleton instance.
	static bool initialize();

	/// Disposes the singleton and free its data.
	static void dispose(int environmentId);

	/// Returns true if the plugin is initialized.
	static bool isInitialized();

	/// Creates a non-tile NavMesh.
	static dtStatus createNavMesh(const NavMeshBuildConfig& config, const float* bmin, const float* bmax,
	                              const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh,
	                              int environmentId);

	/// Disposes the NavMesh passed in parameter.
	static void disposeNavMesh(void* allocatedNavMesh, int environmentId);

	/**
	 * \brief Creates an empty TileNavMesh. Tiles should be built later.
	 * \param config Contains all the parameters that should be used to build the NavMesh.
	 * \param tileSize The size of the tile (m)
	 * \param bmin The min bounds (world coordinates) of the whole navMesh
	 * \param bmax The max bounds (world coordinates) of the whole navMesh
	 * \param allocatedNavMesh The returned allocated NavMesh.
	 * \param tilesNumber The returned tiles number
	 * \param environmentId The environment id the NavMesh is linked to.
	 * \return DT_SUCCESS if success, DT_FAILURE and some other flags if it failed.
	 */
	static dtStatus createTileNavMesh(const NavMeshBuildConfig& config, float tileSize,
	                                  const float* bmin, const float* bmax,
	                                  void*& allocatedNavMesh, int* tilesNumber, int environmentId);

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
	 * \param context The context to use. Used to return the timings to the C# side.
	 */
	static void addTile(const int* tileCoordinates, const NavMeshBuildConfig& config, float tileSize, const float* bmin,
	                    const float* bmax,
	                    const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh, const BlockArea* blockAreas,
	                    int blocksCount, BuildContext* context);

	/// Creates a TileNavMesh by building a ChunkyMesh. Not used anymore for now.
	static dtStatus createTileNavMeshWithChunkyMesh(const NavMeshBuildConfig& config, float tileSize,
	                                                bool buildAllTiles,
	                                                const float* bmin, const float* bmax,
	                                                const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh,
	                                                void*& computedChunkyTriMesh, int* tilesNumber, int environmentId);

	/// Adds a tile to a TileNavMesh by building a ChunkyMesh. Not used anymore for now.
	static void addTileWithChunkyMesh(const int* tileCoordinate, const NavMeshBuildConfig& config, float tileSize,
	                                  const float* bmin, const float* bmax,
	                                  const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh,
	                                  const rcChunkyTriMesh* chunkyMesh, bool dontRecomputeBounds = false);

	/**
	 * \brief Creates a NavMeshQuery
	 * \param navMesh The NavMesh the query should use.
	 * \param maxNodes The maximum nodes that should be used for the path.
	 * \param allocatedNavMeshQuery The created NavMeshQuery
	 * \param environmentId The environment id the NavMesh is linked to.
	 * \return DT_SUCCESS if success, DT_FAILURE and some other flags if it failed.
	 */
	static dtStatus createNavMeshQuery(const void* navMesh, int maxNodes, void*& allocatedNavMeshQuery,
	                                   int environmentId);

	/// Dispose the NavMeshQuery passed in parameter.
	static void disposeNavMeshQuery(void*& allocatedNavMeshQuery, int environmentId);

private:
	/// Made it private so that nobody can call the constructor outside of this class.
	RecastUnityPluginManager()
	{
	}

	/// Free the navMeshes and the navmesh queries for the specified environment.
	void disposeData(int environmentId);

	static dtStatus BuildAllTiles(dtNavMesh* navMesh, const NavMeshBuildConfig& config, float tileSize,
	                              const float* bmin, const float* bmax,
	                              const NavMeshInputGeometry& inputGeometry, const rcChunkyTriMesh* chunkyMesh,
	                              rcContext& context);

	static unsigned char* buildTileMesh(const int tx, const int ty, const NavMeshBuildConfig& config, float tileSize,
	                                    const float* bmin, const float* bmax,
	                                    const NavMeshInputGeometry& inputGeometry, int& dataSize,
	                                    const BlockArea* blockAreas, int blocksCount, rcContext& context);

	static unsigned char* buildTileMeshWithChunkyMesh(const int tx, const int ty, const NavMeshBuildConfig& config,
	                                                  float tileSize,
	                                                  const float* bmin, const float* bmax,
	                                                  const NavMeshInputGeometry& inputGeometry,
	                                                  const rcChunkyTriMesh* chunkyMesh, int& dataSize,
	                                                  rcContext& context);

	/// The instance of the singleton.
	static RecastUnityPluginManager* s_instance;

	/// All the navMeshes that were allocated for the C# side. The NavMesh are sorted by environment id (client X or server)
	std::multimap<int, dtNavMesh*> m_navMeshes;

	/// All the navMesh queries that were allocated for the C# side. The NavMesh queries are sorted by environment id (client X or server)
	std::multimap<int, dtNavMeshQuery*> m_navMeshQueries;
};
#endif
