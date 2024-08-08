#ifndef RECAST_UNITY_PLUGIN_MANAGER_H
#define RECAST_UNITY_PLUGIN_MANAGER_H

#include <vector>

#include "DetourNavMesh.h"
#include "NavMeshBuildConfig.h"
#include "NavMeshBuildData.h"
#include "NavMeshInputGeometry.h"
#include "Recast.h"

// // 1u << 8 to 1u << 28 are free to use
// static const unsigned int DT_HEIGHT_FIELD_FAILED = 1u << 14;
//

// Copy of SamplePartitionType 
enum PartitionType
{
	PARTITION_WATERSHED,
	PARTITION_MONOTONE,
	PARTITION_LAYERS
};

/// A singleton that contains all the allocated data for the C# side.
/// Before using it, you should call RecastUnityPluginManager::Initialize()
class RecastUnityPluginManager
{
public:

	/// Creates the singleton instance.
	static bool initialize();
	
	/// Disposes the singleton and free its data.
	static void dispose();

	static bool isInitialized();
		
	static dtStatus createNavMesh(const NavMeshBuildConfig& config, const float* bmin, const float* bmax,
		const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh);

	static void disposeNavMesh(void* allocatedNavMesh);

	static dtStatus createTileNavMesh(const NavMeshBuildConfig& config, float tileSize, bool buildAllTiles,
		const float* bmin, const float* bmax,
		const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh, void*& computedChunkyTriMesh, int* tilesNumber);

	static void addTile(const int* tileCoordinate, const NavMeshBuildConfig& config, float tileSize, const float* bmin, const float* bmax,
				   const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh, const rcChunkyTriMesh* chunkyMesh, bool dontRecomputeBounds = false);
	
	static dtStatus createNavMeshQuery(const void* allocatedNavMesh, int maxNodes, void*& allocatedNavMeshQuery);

	static void disposeNavMeshQuery(void*& allocatedNavMeshQuery);
	
private:
	/// Made it private so that nobody can call the constructor outside of this class.
	RecastUnityPluginManager() {}

	/// Free the navmeshes and the navmesh queries.
	void disposeData();

	static dtStatus BuildAllTiles(dtNavMesh* navMesh, const NavMeshBuildConfig& config, float tileSize,
	                              const float* bmin, const float* bmax,
	                              const NavMeshInputGeometry& inputGeometry, const rcChunkyTriMesh* chunkyMesh, rcContext& context);

	static unsigned char* buildTileMesh(const int tx, const int ty, const NavMeshBuildConfig& config, float tileSize,
		const float* bmin, const float* bmax,
		const NavMeshInputGeometry& inputGeometry, const rcChunkyTriMesh* chunkyMesh, int& dataSize, rcContext& context);
	
	/// The instance of the singleton.
	static RecastUnityPluginManager* s_instance;

	/// All the navmeshes that were allocated for the C# side.
	std::vector<dtNavMesh*> m_navMeshes;

	/// All the navmesh queries that were allocated for the C# side.
	std::vector<dtNavMeshQuery*> m_navMeshQueries;
};
#endif