#ifndef RECAST_UNITY_PLUGIN_MANAGER_H
#define RECAST_UNITY_PLUGIN_MANAGER_H

#include <vector>

#include "DetourNavMesh.h"
#include "NavMeshBuildConfig.h"
#include "NavMeshInputGeometry.h"

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
	static bool Initialize();
	
	/// Disposes the singleton and free its data.
	static void Dispose();

	static bool IsInitialized();

	static dtStatus CreateNavMesh(const NavMeshBuildConfig& config, const float* bmin, const float* bmax,
		const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh);

	static void DisposeNavMesh(void*& allocatedNavMesh);

	static dtStatus CreateNavMeshQuery(const void* allocatedNavMesh, int maxNodes, void*& allocatedNavMeshQuery);

	static void DisposeNavMeshQuery(void*& allocatedNavMeshQuery);
	
private:
	/// Made it private so that nobody can call the constructor outside of this class.
	RecastUnityPluginManager() {}

	/// Free the navmeshes and the navmesh queries.
	void DisposeData();
	
	/// The instance of the singleton.
	static RecastUnityPluginManager* s_instance;

	/// All the navmeshes that were allocated for the C# side.
	std::vector<dtNavMesh*> m_navMeshes;

	/// All the navmesh queries that were allocated for the C# side.
	std::vector<dtNavMeshQuery*> m_navMeshQueries;
};
#endif