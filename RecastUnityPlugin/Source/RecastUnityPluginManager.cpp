#include "../Include/RecastUnityPluginManager.h"

#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"

RecastUnityPluginManager* RecastUnityPluginManager::s_instance= nullptr;

bool RecastUnityPluginManager::Initialize()
{
	// TODO add a locking mechanism? Even though these methods should be called only on the main thread.
	if (!IsInitialized())
	{
		s_instance = new RecastUnityPluginManager();
		return true;
	}

	return false;
}

bool RecastUnityPluginManager::IsInitialized()
{
	return s_instance != nullptr;
}

void RecastUnityPluginManager::Dispose()
{
	// TODO add a locking mechanism? Even though these methods should be called only on the main thread.
	if (IsInitialized())
	{
		s_instance->DisposeData();
		delete s_instance;
		s_instance = nullptr;
	}
}

void RecastUnityPluginManager::DisposeData()
{
	for (auto navmesh : m_navMeshes)
	{
		dtFreeNavMesh(navmesh);
	}
	m_navMeshes.clear();	

	for (auto query : m_navMeshQueries)
	{
		dtFreeNavMeshQuery(query);
	}
	m_navMeshQueries.clear();	
}

// TODO: Make the tiled mesh version
// More or less copied from Sample_SoloMesh.cpp
dtStatus RecastUnityPluginManager::CreateNavMesh(const NavMeshBuildConfig config, const float* bmin, const float* bmax,
	const float* verts, int nverts, const int* tris, int ntris,	void*& allocatedNavMesh)
{
	if (!IsInitialized())
	{
		return DT_FAILURE;
	}
	
	//
	// Step 1. Initialize build config.
	//

	rcConfig rcConfig;
	rcConfig.cs = config.cs;
	rcConfig.ch = config.ch;
	rcConfig.walkableSlopeAngle = config.walkableSlopeAngle;
	rcConfig.walkableHeight = (int)ceilf(config.agentHeight / config.ch);
	rcConfig.walkableClimb = (int)floorf(config.agentMaxClimb / config.ch);
	rcConfig.walkableRadius = (int)ceilf(config.agentRadius / config.cs);
	rcConfig.maxEdgeLen = (int)((float)config.maxEdgeLen / config.cs);
	rcConfig.maxSimplificationError = config.maxSimplificationError;
	rcConfig.minRegionArea = (int)config.minRegionArea ;		// Note: area = size*size
	rcConfig.mergeRegionArea = (int)config.mergeRegionArea;	// Note: area = size*size
	rcConfig.maxVertsPerPoly = (int)config.maxVertsPerPoly;
	rcConfig.detailSampleDist = config.detailSampleDist < 0.9f ? 0 : config.cs * config.detailSampleDist;
	rcConfig.detailSampleMaxError = config.ch * config.detailSampleMaxError;
	
	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	rcVcopy(rcConfig.bmin, bmin);
	rcVcopy(rcConfig.bmax, bmax);
	rcCalcGridSize(rcConfig.bmin, rcConfig.bmax, rcConfig.cs, &rcConfig.width, &rcConfig.height);
	
	rcContext context;
	
	//
	// Step 2. Rasterize input polygon soup.
	//
	// Allocate voxel heightfield where we rasterize our input data to.
	rcHeightfield* solid = rcAllocHeightfield();
	if (!solid)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return DT_FAILURE;
	}
	if (!rcCreateHeightfield(&context, *solid, rcConfig.width, rcConfig.height, rcConfig.bmin, rcConfig.bmax, rcConfig.cs, rcConfig.ch))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return DT_FAILURE;
	}
	
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	unsigned char* triareas = new unsigned char[ntris];
	if (!triareas)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}
	
	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle, verts, nverts, tris, ntris, triareas);
	if (!rcRasterizeTriangles(&context, verts, nverts, tris, triareas, ntris, *solid, rcConfig.walkableClimb))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	delete [] triareas;
	triareas = nullptr;
	
	//
	// Step 3. Filter walkable surfaces.
	//
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (config.filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(&context, rcConfig.walkableClimb, *solid);
	if (config.filterLedgeSpans)
		rcFilterLedgeSpans(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *solid);
	if (config.filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(&context, rcConfig.walkableHeight, *solid);

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	rcCompactHeightfield* chf = rcAllocCompactHeightfield();
	if (!chf)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *solid, *chf))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}
	
	rcFreeHeightField(solid);
	solid = nullptr;

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(&context, rcConfig.walkableRadius, *chf))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// TODO: for now there is nothing to mark areas, maybe later.
	// (Optional) Mark areas.
	// const ConvexVolume* vols = m_geom->getConvexVolumes();
	// for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	// 	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 partitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the navmesh, use this if you have large open areas
	// 2) Monotone partitioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (config.partitionType == PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(&context, *chf))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(&context, *chf, 0, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (config.partitionType == PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(&context, *chf, 0, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(&context, *chf, 0, rcConfig.minRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	// Create contours.
	rcContourSet* cset = rcAllocContourSet();
	if (!cset)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(&context, *chf, rcConfig.maxSimplificationError, rcConfig.maxEdgeLen, *cset))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//
	
	// Build polygon navmesh from the contours.
	rcPolyMesh* pmesh = rcAllocPolyMesh();
	if (!pmesh)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(&context, *cset, rcConfig.maxVertsPerPoly, *pmesh))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}
	
	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
	if (!dmesh)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(&context, *pmesh, *chf, rcConfig.detailSampleDist, rcConfig.detailSampleMaxError, *dmesh))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}
	
	rcFreeCompactHeightfield(chf);
	chf = nullptr;
	rcFreeContourSet(cset);
	cset = nullptr;

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	unsigned char* navData = 0;
	int navDataSize = 0;

	
	// Update poly flags from areas.
	for (int i = 0; i < pmesh->npolys; ++i)
	{
		// We have to set a flag different from 0 for the tiles, otherwise it won't work (PassFilter will always return false)
		pmesh->flags[i] = 1;
		// TODO: For now we don't set specific flags to areas
		// if (pmesh->areas[i] == RC_WALKABLE_AREA)
		// 	pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
		// 	
		// if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
		// 	m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
		// 	m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
		// {
		// 	m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
		// }
		// else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
		// {
		// 	m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
		// }
		// else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
		// {
		// 	m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
		// }
	}

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = pmesh->verts;
	params.vertCount = pmesh->nverts;
	params.polys = pmesh->polys;
	params.polyAreas = pmesh->areas;
	params.polyFlags = pmesh->flags;
	params.polyCount = pmesh->npolys;
	params.nvp = pmesh->nvp;
	params.detailMeshes = dmesh->meshes;
	params.detailVerts = dmesh->verts;
	params.detailVertsCount = dmesh->nverts;
	params.detailTris = dmesh->tris;
	params.detailTriCount = dmesh->ntris;

	// No offmesh connextions for now
	params.offMeshConVerts = nullptr;
	params.offMeshConRad = nullptr;
	params.offMeshConDir = nullptr;
	params.offMeshConAreas = nullptr;
	params.offMeshConFlags = nullptr;
	params.offMeshConUserID = nullptr;
	params.offMeshConCount = 0;
	
	params.walkableHeight = config.agentHeight;
	params.walkableRadius = config.agentRadius;
	params.walkableClimb = config.agentMaxClimb;
	rcVcopy(params.bmin, pmesh->bmin);
	rcVcopy(params.bmax, pmesh->bmax);
	params.cs = rcConfig.cs;
	params.ch = rcConfig.ch;
	params.buildBvTree = true;
	
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
	{
		context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
		return false;
	}
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		dtFree(navData);
		context.log(RC_LOG_ERROR, "Could not create Detour navmesh");
		return false;
	}
	
	dtStatus status;
	
	status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(navData);
		context.log(RC_LOG_ERROR, "Could not init Detour navmesh");
		return false;
	}

	// Cleanup
	rcFreePolyMesh(pmesh);
	pmesh = nullptr;
	rcFreePolyMeshDetail(dmesh);
	dmesh = nullptr;
	
	// Store the allocated navmesh.
	s_instance->m_navMeshes.push_back(navMesh);

	allocatedNavMesh = navMesh;
	
	return DT_SUCCESS;
}

void RecastUnityPluginManager::DisposeNavMesh(void*& allocatedNavMesh)
{
	if (allocatedNavMesh == nullptr)
	{
		return;
	}
	
	auto navMesh = (dtNavMesh*)allocatedNavMesh;
	auto navMeshIterator = std::find(s_instance->m_navMeshes.begin(), s_instance->m_navMeshes.end(), navMesh);
	if (navMeshIterator != s_instance->m_navMeshes.end())
	{
		s_instance->m_navMeshes.erase(navMeshIterator);
	}

	dtFreeNavMesh(navMesh);
	allocatedNavMesh = nullptr;
}


dtStatus RecastUnityPluginManager::CreateNavMeshQuery(const void* allocatedNavMesh, int maxNodes, void*& allocatedNavMeshQuery)
{
	auto navMeshQuery = dtAllocNavMeshQuery();
	if (navMeshQuery != nullptr)
	{
		// Store the allocated navmesh query.
		s_instance->m_navMeshQueries.push_back(navMeshQuery);
		navMeshQuery->init((const dtNavMesh*)allocatedNavMesh, maxNodes);
		allocatedNavMeshQuery = navMeshQuery;
		return DT_SUCCESS;
	}

	return DT_FAILURE;
}

void RecastUnityPluginManager::DisposeNavMeshQuery(void*& allocatedNavMeshQuery)
{
	if (allocatedNavMeshQuery == nullptr)
	{
		return;
	}
	
	auto navMeshQuery = (dtNavMeshQuery*)allocatedNavMeshQuery;
	auto navMeshQueryIterator = std::find(s_instance->m_navMeshQueries.begin(), s_instance->m_navMeshQueries.end(), navMeshQuery);
	if (navMeshQueryIterator != s_instance->m_navMeshQueries.end())
	{
		s_instance->m_navMeshQueries.erase(navMeshQueryIterator);
	}

	dtFreeNavMeshQuery(navMeshQuery);
	allocatedNavMeshQuery = nullptr;
}

