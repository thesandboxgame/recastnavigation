#include "../Include/RecastUnityPluginManager.h"

#include "ChunkyTriMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"

RecastUnityPluginManager* RecastUnityPluginManager::s_instance= nullptr;

inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}


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

// More or less copied from Sample_SoloMesh.cpp
dtStatus RecastUnityPluginManager::CreateNavMesh(const NavMeshBuildConfig& config, const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh)
{
	if (!IsInitialized())
	{
		return DT_FAILURE;
	}

	const float* verts = inputGeometry.vertices;
	int nverts = inputGeometry.verticesCount;
	const int* tris = inputGeometry.triangles;
	int ntris = inputGeometry.trianglesCount;
	
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
		rcFreeHeightField(solid);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return DT_FAILURE;
	}
	
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	unsigned char* triareas = new unsigned char[ntris];
	if (!triareas)
	{
		rcFreeHeightField(solid);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return DT_FAILURE;
	}
	
	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle, verts, nverts, tris, ntris, triareas);
	if (!rcRasterizeTriangles(&context, verts, nverts, tris, triareas, ntris, *solid, rcConfig.walkableClimb))
	{
		rcFreeHeightField(solid);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return DT_FAILURE;
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
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return DT_FAILURE;
	}
	if (!rcBuildCompactHeightfield(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *solid, *chf))
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return DT_FAILURE;
	}
	
	rcFreeHeightField(solid);
	solid = nullptr;

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(&context, rcConfig.walkableRadius, *chf))
	{
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return DT_FAILURE;
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
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return DT_FAILURE;
		}
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(&context, *chf, 0, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return DT_FAILURE;
		}
	}
	else if (config.partitionType == PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(&context, *chf, 0, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return DT_FAILURE;
		}
	}
	else // PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(&context, *chf, 0, rcConfig.minRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return DT_FAILURE;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	// Create contours.
	rcContourSet* cset = rcAllocContourSet();
	if (!cset)
	{
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return DT_FAILURE;
	}
	if (!rcBuildContours(&context, *chf, rcConfig.maxSimplificationError, rcConfig.maxEdgeLen, *cset))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return DT_FAILURE;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//
	
	// Build polygon navmesh from the contours.
	rcPolyMesh* pmesh = rcAllocPolyMesh();
	if (!pmesh)
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return DT_FAILURE;
	}
	if (!rcBuildPolyMesh(&context, *cset, rcConfig.maxVertsPerPoly, *pmesh))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return DT_FAILURE;
	}
	
	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
	if (!dmesh)
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return DT_FAILURE;
	}

	if (!rcBuildPolyMeshDetail(&context, *pmesh, *chf, rcConfig.detailSampleDist, rcConfig.detailSampleMaxError, *dmesh))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return DT_FAILURE;
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
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
		return DT_FAILURE;
	}
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		dtFree(navData);
		context.log(RC_LOG_ERROR, "Could not create Detour navmesh");
		return DT_FAILURE;
	}
	
	dtStatus status;
	
	status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		dtFree(navData);
		context.log(RC_LOG_ERROR, "Could not init Detour navmesh");
		return DT_FAILURE;
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

// More or less copied from Sample_TileMesh.cpp
dtStatus RecastUnityPluginManager::CreateTileNavMesh(const NavMeshBuildConfig& config, float tileSize, bool buildAllTiles,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh)
{
	if (!IsInitialized())
	{
		return DT_FAILURE;
	}

	rcContext context;
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}
	
	dtNavMeshParams params;
	rcVcopy(params.orig, bmin);
	params.tileWidth = tileSize * config.cs;
	params.tileHeight = tileSize * config.cs;

	// Copied from Sample_TileMesh::handleSettings
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const int ts = (int)tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	int maxTiles = 1 << tileBits;
	int maxPolysPerTile = 1 << polyBits;
	params.maxTiles = maxTiles;
	params.maxPolys = maxPolysPerTile;
	
	dtStatus status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		dtFree(navMesh);
		return DT_FAILURE;
	}

	if (buildAllTiles)
	{
		dtStatus tilesCreationStatus = BuildAllTiles(navMesh, config, tileSize, bmin, bmax, inputGeometry, context);
		if (dtStatusFailed(tilesCreationStatus))
		{
			context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
			dtFree(navMesh);
			return DT_FAILURE;
		}
	}

	// Store the allocated navmesh.
	s_instance->m_navMeshes.push_back(navMesh);

	allocatedNavMesh = navMesh;
		
	return DT_SUCCESS;
}

// copied from Sample_TileMesh::buildAllTiles()
dtStatus RecastUnityPluginManager::BuildAllTiles(dtNavMesh* navMesh, const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, rcContext& context)
{
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const int ts = (int)tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	const float tcs = tileSize * config.cs;

	float lastBuiltTileBmin[3];
	float lastBuiltTileBmax[3];

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			lastBuiltTileBmin[1] = bmin[1];
			lastBuiltTileBmin[2] = bmin[2] + y*tcs;
			
			lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
			lastBuiltTileBmax[1] = bmax[1];
			lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;
			
			int dataSize = 0;
			unsigned char* data = BuildTileMesh(x, y, navMesh, config, tileSize, lastBuiltTileBmin, lastBuiltTileBmax, inputGeometry, dataSize, context);
			if (data)
			{
				// TODO: we should not need to remove the previous data, because there should not be one.
				// // Remove any previous data (navmesh owns and deletes the data).
				// navMesh->removeTile(navMesh->getTileRefAt(x,y,0),0,0);
				// Let the navmesh own the data.
				dtStatus status = navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
					// TODO : do we need to return here in case there is a failure?
					dtFree(data);
			}
		}
	}
	
	return DT_SUCCESS;
}

unsigned char* RecastUnityPluginManager::BuildTileMesh(const int tx, const int ty, dtNavMesh* navMesh, const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, int& dataSize, rcContext& context)
{
	const float* verts = inputGeometry.vertices;
	int nverts = inputGeometry.verticesCount;
	const int* tris = inputGeometry.triangles;
	int ntris = inputGeometry.trianglesCount;

	rcChunkyTriMesh* chunkyMesh = new rcChunkyTriMesh;
	if (!chunkyMesh)
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return nullptr;
	}
	if (!rcCreateChunkyTriMesh(verts, tris, ntris, 256, chunkyMesh))
	{
		delete chunkyMesh;
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
		return nullptr;
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
	rcConfig.tileSize = (int)tileSize;
	rcConfig.borderSize = rcConfig.walkableRadius + 3;// Reserve enough padding.
	rcConfig.width = rcConfig.tileSize + rcConfig.borderSize*2;
	rcConfig.height = rcConfig.tileSize + rcConfig.borderSize*2;
	rcConfig.detailSampleDist = config.detailSampleDist < 0.9f ? 0 : config.cs * config.detailSampleDist;
	rcConfig.detailSampleMaxError = config.ch * config.detailSampleMaxError;
	
	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// No polygons (or contours) will be created on the border area.
	//
	// IMPORTANT!
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build
	//   : |     | :  
	//   : +-----+ :<-- geometry needed
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	rcVcopy(rcConfig.bmin, bmin);
	rcVcopy(rcConfig.bmax, bmax);
	rcConfig.bmin[0] -= rcConfig.borderSize*rcConfig.cs;
	rcConfig.bmin[2] -= rcConfig.borderSize*rcConfig.cs;
	rcConfig.bmax[0] += rcConfig.borderSize*rcConfig.cs;
	rcConfig.bmax[2] += rcConfig.borderSize*rcConfig.cs;
	
	//
	// Step 2. Rasterize input polygon soup.
	//
	// Allocate voxel heightfield where we rasterize our input data to.
	rcHeightfield* solid = rcAllocHeightfield();
	if (!solid)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return nullptr;
	}
	if (!rcCreateHeightfield(&context, *solid, rcConfig.width, rcConfig.height, rcConfig.bmin, rcConfig.bmax, rcConfig.cs, rcConfig.ch))
	{
		rcFreeHeightField(solid);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return nullptr;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	unsigned char* triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!triareas)
	{
		rcFreeHeightField(solid);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return nullptr;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = rcConfig.bmin[0];
	tbmin[1] = rcConfig.bmin[2];
	tbmax[0] = rcConfig.bmax[0];
	tbmax[1] = rcConfig.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		rcFreeHeightField(solid);
		return nullptr;
	}

	int tileTriCount = 0;
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* ctris = &chunkyMesh->tris[node.i*3];
		const int nctris = node.n;
		
		tileTriCount += nctris;
		
		memset(triareas, 0, nctris*sizeof(unsigned char));
		rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle,
								verts, nverts, ctris, nctris, triareas);
		
		if (!rcRasterizeTriangles(&context, verts, nverts, ctris, triareas, nctris, *solid, rcConfig.walkableClimb))
		{
			rcFreeHeightField(solid);
			return nullptr;
		}
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
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return nullptr;
	}
	if (!rcBuildCompactHeightfield(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *solid, *chf))
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return nullptr;
	}

	rcFreeHeightField(solid);
	solid = nullptr;

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(&context, rcConfig.walkableRadius, *chf))
	{
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return nullptr;
	}

	// TODO: for now there is nothing to mark areas, maybe later.
	// (Optional) Mark areas.
	// const ConvexVolume* vols = m_geom->getConvexVolumes();
	// for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	// 	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
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
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return nullptr;
		}
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(&context, *chf, rcConfig.borderSize, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return nullptr;
		}
	}
	else if (config.partitionType == PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(&context, *chf, rcConfig.borderSize, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return nullptr;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(&context, *chf, rcConfig.borderSize, rcConfig.minRegionArea))
		{
			rcFreeCompactHeightfield(chf);
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return nullptr;
		}
	}

	// Create contours.
	rcContourSet* cset = rcAllocContourSet();
	if (!cset)
	{
		rcFreeCompactHeightfield(chf);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return nullptr;
	}
	if (!rcBuildContours(&context, *chf, rcConfig.maxSimplificationError, rcConfig.maxEdgeLen, *cset))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return nullptr;
	}
	
	if (cset->nconts == 0)
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		return nullptr;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	rcPolyMesh* pmesh = rcAllocPolyMesh();
	if (!pmesh)
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return nullptr;
	}
	if (!rcBuildPolyMesh(&context, *cset, rcConfig.maxVertsPerPoly, *pmesh))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return nullptr;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
	if (!dmesh)
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return nullptr;
	}

	if (!rcBuildPolyMeshDetail(&context, *pmesh, *chf, rcConfig.detailSampleDist, rcConfig.detailSampleMaxError, *dmesh))
	{
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return nullptr;
	}

	rcFreeCompactHeightfield(chf);
	chf = nullptr;
	rcFreeContourSet(cset);
	cset = nullptr;

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (rcConfig.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			context.log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", pmesh->nverts, 0xffff);
			rcFreePolyMesh(pmesh);
			rcFreePolyMeshDetail(dmesh);
			return nullptr;
		}
		
		// Update poly flags from areas.
		for (int i = 0; i < pmesh->npolys; ++i)
		{
			// We have to set a flag different from 0 for the tiles, otherwise it won't work (PassFilter will always return false)
			pmesh->flags[i] = 1;
			// TODO: For now we don't set specific flags to areas
			// if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
			// 	m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
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
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, pmesh->bmin);
		rcVcopy(params.bmax, pmesh->bmax);
		params.cs = rcConfig.cs;
		params.ch = rcConfig.ch;
		params.buildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			rcFreePolyMesh(pmesh);
			rcFreePolyMeshDetail(dmesh);
			context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return nullptr;
		}		
	}
	
	dataSize = navDataSize;
	return navData;
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

