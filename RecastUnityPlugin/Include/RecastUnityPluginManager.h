#ifndef RECAST_UNITY_PLUGIN_MANAGER_H
#define RECAST_UNITY_PLUGIN_MANAGER_H

#include <vector>

#include "BlockArea.h"
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

// This is used to mark polygons when building it.
enum PolyAreas
{
	POLYAREA_DEFAULT = 0x01,
	POLYAREA_LIQUID = 0x02,
	// SAMPLE_POLYAREA_ROAD,
	// SAMPLE_POLYAREA_DOOR,
	// SAMPLE_POLYAREA_GRASS,
	// SAMPLE_POLYAREA_JUMP
};

// This is used by the Nav Mesh queries to know if the agent has to ability to cross some polygons.
enum PolyFlags
{
	POLYFLAGS_WALK			= 0x01,		// Default area type. Can walk on it (ground, grass, etc.)
	POLYFLAGS_SWIM		= 0x02,		// Area type for liquids. Requires to swim (water, lava, etc.).
	// SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	// SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	// SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL	= 0xffff	// All abilities.
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

	static dtStatus createTileNavMesh(const NavMeshBuildConfig& config, float tileSize,
		const float* bmin, const float* bmax,
		void*& allocatedNavMesh, int* tilesNumber);
	
	static void addTile(const int* tileCoordinate, const NavMeshBuildConfig& config, float tileSize, const float* bmin, const float* bmax,
			   const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh, const BlockArea* blockAreas, int blocksCount);
	
	static dtStatus createTileNavMeshWithChunkyMesh(const NavMeshBuildConfig& config, float tileSize, bool buildAllTiles,
		const float* bmin, const float* bmax,
		const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh, void*& computedChunkyTriMesh, int* tilesNumber);

	static void addTileWithChunkyMesh(const int* tileCoordinate, const NavMeshBuildConfig& config, float tileSize, const float* bmin, const float* bmax,
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
		const NavMeshInputGeometry& inputGeometry, int& dataSize, const BlockArea* blockAreas, int blocksCount, rcContext& context);
	
	static unsigned char* buildTileMeshWithChunkyMesh(const int tx, const int ty, const NavMeshBuildConfig& config, float tileSize,
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