#include "../Include/RecastUnityPluginManager.h"

#include "ChunkyTriMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "NavMeshBuildData.h"
#include "NavMeshBuildUtility.h"
#include "Recast.h"
#include <math.h>
#include <cstring>
#include <algorithm>

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


bool RecastUnityPluginManager::initialize()
{
	// TODO add a locking mechanism? Even though these methods should be called only on the main thread.
	if (!isInitialized())
	{
		s_instance = new RecastUnityPluginManager();
		return true;
	}

	return false;
}

bool RecastUnityPluginManager::isInitialized()
{
	return s_instance != nullptr;
}

void RecastUnityPluginManager::dispose(int environmentId)
{
	// TODO add a locking mechanism? Even though these methods should be called only on the main thread.
	if (isInitialized())
	{
		s_instance->disposeData(environmentId);
		if (s_instance->m_navMeshes.empty())
		{
			delete s_instance;
			s_instance = nullptr;
		}
	}
}

void RecastUnityPluginManager::disposeData(int environmentId)
{
	for (auto entry : m_navMeshes)
	{
		if (entry.first == environmentId)
		{
			dtFreeNavMesh(entry.second);
		}
	}
	m_navMeshes.erase(environmentId);

	for (auto entry : m_navMeshQueries)
	{
		if (entry.first == environmentId)
		{
			dtFreeNavMeshQuery(entry.second);
		}
	}
	m_navMeshQueries.erase(environmentId);	
}

// More or less copied from Sample_SoloMesh.cpp
dtStatus RecastUnityPluginManager::createNavMesh(const NavMeshBuildConfig& config, const float* bmin, const float* bmax,
                                                 const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh, int environmentId)
{
	if (!isInitialized())
	{
		return DT_FAILURE;
	}

	NavMeshBuildData buildData;
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
	
	if (NavMeshBuildUtility::prepareTriangleRasterization(rcConfig, ntris, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}
	
	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(buildData.triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle, verts, nverts, tris, ntris, buildData.triareas);
	if (!rcRasterizeTriangles(&context, verts, nverts, tris, buildData.triareas, ntris, *buildData.solid, rcConfig.walkableClimb))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return DT_FAILURE;
	}
	
	//
	// Step 3. Filter walkable surfaces.
	//

	NavMeshBuildUtility::filterWalkingSurfaces(config.filterLowHangingObstacles, config.filterLedgeSpans, config.filterWalkableLowHeightSpans,
		rcConfig, buildData, context);
	
	//
	// Step 4. Partition walkable surface to simple regions.
	//
	
	if (NavMeshBuildUtility::preparePartionning(rcConfig, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}
	
	// TODO: for now there is nothing to mark areas, maybe later.
	// (Optional) Mark areas.
	// const ConvexVolume* vols = m_geom->getConvexVolumes();
	// for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	// 	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	if (NavMeshBuildUtility::buildRegions(config.partitionType, 0, rcConfig, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	if (NavMeshBuildUtility::createContours(rcConfig, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//
	
	if (NavMeshBuildUtility::buildMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}
	
	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	
	if (NavMeshBuildUtility::buildDetailMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return DT_FAILURE;
	}
	
	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	buildData.navData = 0;
	int navDataSize = 0;
	
	// Update poly flags from areas.
	for (int i = 0; i < buildData.pmesh->npolys; ++i)
	{
		// We have to set a flag different from 0 for the tiles, otherwise it won't work (PassFilter will always return false)
		buildData.pmesh->flags[i] = 1;
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
	params.verts = buildData.pmesh->verts;
	params.vertCount = buildData.pmesh->nverts;
	params.polys = buildData.pmesh->polys;
	params.polyAreas = buildData.pmesh->areas;
	params.polyFlags = buildData.pmesh->flags;
	params.polyCount = buildData.pmesh->npolys;
	params.nvp = buildData.pmesh->nvp;
	params.detailMeshes = buildData.dmesh->meshes;
	params.detailVerts = buildData.dmesh->verts;
	params.detailVertsCount = buildData.dmesh->nverts;
	params.detailTris = buildData.dmesh->tris;
	params.detailTriCount = buildData.dmesh->ntris;

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
	rcVcopy(params.bmin, buildData.pmesh->bmin);
	rcVcopy(params.bmax, buildData.pmesh->bmax);
	params.cs = rcConfig.cs;
	params.ch = rcConfig.ch;
	params.buildBvTree = true;
	
	if (!dtCreateNavMeshData(&params, &buildData.navData, &navDataSize))
	{
		context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
		return DT_FAILURE;
	}
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		context.log(RC_LOG_ERROR, "Could not create Detour navmesh");
		return DT_FAILURE;
	}
	
	dtStatus status;
	
	status = navMesh->init(buildData.navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		context.log(RC_LOG_ERROR, "Could not init Detour navmesh");
		return DT_FAILURE;
	}

	// The nav data should not be disposed
	buildData.ownsNavData = false;
	
	// Store the allocated navmesh.
	s_instance->m_navMeshes.insert({environmentId, navMesh});

	allocatedNavMesh = navMesh;
	
	return DT_SUCCESS;
}

// More or less copied from Sample_TileMesh.cpp
dtStatus RecastUnityPluginManager::createTileNavMesh(const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	void*& allocatedNavMesh, int* tilesNumber, int environmentId)
{
		if (!isInitialized())
	{
		return DT_FAILURE;
	}

	rcContext context;
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return DT_FAILURE;
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
	tilesNumber[0] = tw;
	tilesNumber[1] = th;

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
	
	// Store the allocated navmesh.
	s_instance->m_navMeshes.insert({environmentId, navMesh});

	allocatedNavMesh = navMesh;
		
	return DT_SUCCESS;
}


dtStatus RecastUnityPluginManager:: createTileNavMeshWithChunkyMesh(const NavMeshBuildConfig& config, float tileSize, bool buildAllTiles,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, void*& allocatedNavMesh, void*& computedChunkyTriMesh, int* tilesNumber, int environmentId)
{
	if (!isInitialized())
	{
		return DT_FAILURE;
	}

	rcContext context;
	
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return DT_FAILURE;
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
	tilesNumber[0] = tw;
	tilesNumber[1] = th;

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

	rcChunkyTriMesh* chunkyMesh = new rcChunkyTriMesh;
	if (!chunkyMesh)
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return DT_FAILURE;
	}

	const float* verts = inputGeometry.vertices;
	const int* tris = inputGeometry.triangles;
	int ntris = inputGeometry.trianglesCount;
	
	if (!rcCreateChunkyTriMesh(verts, tris, ntris, 256, chunkyMesh))
	{
		context.log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
		delete chunkyMesh;
		return DT_FAILURE;
	}

	if (buildAllTiles)
	{
		dtStatus tilesCreationStatus = BuildAllTiles(navMesh, config, tileSize, bmin, bmax, inputGeometry, chunkyMesh, context);
		if (dtStatusFailed(tilesCreationStatus))
		{
			context.log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
			dtFree(navMesh);
			delete chunkyMesh;
			return DT_FAILURE;
		}
	}
	else
	{
		computedChunkyTriMesh = chunkyMesh;
	}

	// Store the allocated navmesh.
	s_instance->m_navMeshes.insert({environmentId, navMesh});

	allocatedNavMesh = navMesh;
		
	return DT_SUCCESS;
}

void RecastUnityPluginManager::addTile(const int* tileCoordinates, const NavMeshBuildConfig& config, float tileSize, const float* bmin, const float* bmax,
		   const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh, const BlockArea* blockAreas, int blocksCount, BuildContext* context)
{
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const float tcs = tileSize * config.cs;

	float lastBuiltTileBmin[3];
	float lastBuiltTileBmax[3];

	int x = tileCoordinates[0];
	int y = tileCoordinates[1];

	// TODO: expose some method to get these bounds in the C# side.
	lastBuiltTileBmin[0] = bmin[0] + x*tcs;
	lastBuiltTileBmin[1] = bmin[1];
	lastBuiltTileBmin[2] = bmin[2] + y*tcs;
		
	lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
	lastBuiltTileBmax[1] = bmax[1];
	lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;
	
	int dataSize = 0;

	unsigned char* data = buildTileMesh(x, y, config, tileSize, lastBuiltTileBmin, lastBuiltTileBmax,
		inputGeometry, dataSize, blockAreas, blocksCount, *context);
	if (data)
	{
		// TODO: we should not need to remove the previous data, because there should not be one.
		// Remove any previous data (navmesh owns and deletes the data).
		navMesh->removeTile(navMesh->getTileRefAt(x,y,0),0,0);
		// Let the navmesh own the data.
		navMesh->mutex.lock();
		dtStatus status = navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			// TODO : do we need to return here in case there is a failure?
			dtFree(data);
		navMesh->mutex.unlock();
	}

	context->computeAllTimings();
}

void RecastUnityPluginManager::addTileWithChunkyMesh(const int* tileCoordinate, const NavMeshBuildConfig& config, float tileSize, const float* bmin, const float* bmax,
				   const NavMeshInputGeometry& inputGeometry, dtNavMesh* navMesh, const rcChunkyTriMesh* chunkyMesh, bool dontRecomputeBounds)
{
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, config.cs, &gw, &gh);
	const float tcs = tileSize * config.cs;

	float lastBuiltTileBmin[3];
	float lastBuiltTileBmax[3];

	int x = tileCoordinate[0];
	int y = tileCoordinate[1];

	if (!dontRecomputeBounds)
	{
		// TODO: remove the code duplication
		lastBuiltTileBmin[0] = bmin[0] + x*tcs;
		lastBuiltTileBmin[1] = bmin[1];
		lastBuiltTileBmin[2] = bmin[2] + y*tcs;
			
		lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
		lastBuiltTileBmax[1] = bmax[1];
		lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;
	}
	else
	{
		lastBuiltTileBmin[0] = bmin[0];
		lastBuiltTileBmin[1] = bmin[1];
		lastBuiltTileBmin[2] = bmin[2];
			
		lastBuiltTileBmax[0] = bmax[0];
		lastBuiltTileBmax[1] = bmax[1];
		lastBuiltTileBmax[2] = bmax[2];
	}

	rcContext context;	
	int dataSize = 0;

	unsigned char* data = buildTileMeshWithChunkyMesh(x, y, config, tileSize, lastBuiltTileBmin, lastBuiltTileBmax,
		inputGeometry, chunkyMesh, dataSize, context);
	if (data)
	{
		// TODO: we should not need to remove the previous data, because there should not be one.
		// Remove any previous data (navmesh owns and deletes the data).
		navMesh->removeTile(navMesh->getTileRefAt(x,y,0),0,0);
		// Let the navmesh own the data.
		navMesh->mutex.lock();
		dtStatus status = navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			// TODO : do we need to return here in case there is a failure?
			dtFree(data);
		navMesh->mutex.unlock();
	}
}

// copied from Sample_TileMesh::buildAllTiles()
dtStatus RecastUnityPluginManager::BuildAllTiles(dtNavMesh* navMesh, const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, const rcChunkyTriMesh* chunkyMesh, rcContext& context)
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
			// TODO: remove the code duplication
			lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			lastBuiltTileBmin[1] = bmin[1];
			lastBuiltTileBmin[2] = bmin[2] + y*tcs;
			
			lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
			lastBuiltTileBmax[1] = bmax[1];
			lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;
			
			int dataSize = 0;
			unsigned char* data = buildTileMeshWithChunkyMesh(x, y, config, tileSize, lastBuiltTileBmin, lastBuiltTileBmax, inputGeometry, chunkyMesh, dataSize, context);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				navMesh->removeTile(navMesh->getTileRefAt(x,y,0),0,0);
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

unsigned char* RecastUnityPluginManager::buildTileMesh(const int tx, const int ty, const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, int& dataSize, const BlockArea* blockAreas, int blocksCount, rcContext& context)
{
	NavMeshBuildData buildData;
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
	
	if (NavMeshBuildUtility::prepareTriangleRasterization(rcConfig, ntris, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}
	
	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(buildData.triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle, verts, nverts, tris, ntris, buildData.triareas);
	if (!rcRasterizeTriangles(&context, verts, nverts, tris, buildData.triareas, ntris, *buildData.solid, rcConfig.walkableClimb))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return nullptr;
	}

	//
	// Step 3. Filter walkable surfaces.
	//

	NavMeshBuildUtility::filterWalkingSurfaces(config.filterLowHangingObstacles, config.filterLedgeSpans, config.filterWalkableLowHeightSpans,
		rcConfig, buildData, context);

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	if (NavMeshBuildUtility::preparePartionning(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}
	
	// (Optional) Mark areas.
	for (int i  = 0; i < blocksCount; ++i)
	{
		// In theory, 4 points would have been enough, but we subdivise the square because the poly marking checks
		// the vertices, not the bounds. Subdividing increase the accuracy of the marking.
		const int blockVerticesCount = 8;
		const float halfSize = 0.5f;
		float vertices[blockVerticesCount * 3];
		const float* center = blockAreas[i].center;
		float minY = center[1] - halfSize;
		// We add an offset to make sure the volume is really taken into account.
		float maxY = center[1] + halfSize * 2;

		// Create the vertices for the bottom plane of the block.
		float* currentVertex = vertices;
		currentVertex[0] = center[0] - halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] - halfSize;

		currentVertex += 3;
		currentVertex[0] = center[0];
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] - halfSize;
		
		currentVertex += 3;
		currentVertex[0] = center[0] + halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] - halfSize;

		currentVertex += 3;
		currentVertex[0] = center[0] + halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2];
		
		currentVertex += 3;
		currentVertex[0] = center[0] + halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] + halfSize;

		currentVertex += 3;
		currentVertex[0] = center[0];
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] + halfSize;

		currentVertex += 3;
		currentVertex[0] = center[0] - halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2] + halfSize;
		
		currentVertex += 3;
		currentVertex[0] = center[0] - halfSize;
		currentVertex[1] = center[1] - halfSize;
		currentVertex[2] = center[2];

		const float* verts = vertices;
		// TODO: maybe do a better methods that checks the bounds or something? Checking the vertices is not very effecient to have an accurate marking.
		rcMarkConvexPolyArea(&context, verts, blockVerticesCount, minY, maxY, (unsigned char)blockAreas[i].area, *buildData.chf);
	}
	
	if (NavMeshBuildUtility::buildRegions(config.partitionType, rcConfig.borderSize, rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	if (NavMeshBuildUtility::createContours(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}
	
	if (buildData.cset->nconts == 0)
	{
		return nullptr;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	if (NavMeshBuildUtility::buildMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	
	if (NavMeshBuildUtility::buildDetailMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	buildData.navData = 0;
	int navDataSize = 0;
	if (rcConfig.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (buildData.pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			context.log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", buildData.pmesh->nverts, 0xffff);
			return nullptr;
		}
		
		// Update poly flags from areas.
		for (int i = 0; i < buildData.pmesh->npolys; ++i)
		{
			// RC_WALKABLE_AREA is the default.
			if (buildData.pmesh->areas[i] == RC_WALKABLE_AREA)
				buildData.pmesh->areas[i] = POLYAREA_DEFAULT;
			
			if (buildData.pmesh->areas[i] == POLYAREA_DEFAULT)
			{
				buildData.pmesh->flags[i] = POLYFLAGS_WALK;
			}
			else if (buildData.pmesh->areas[i] == POLYAREA_LIQUID)
			{
				buildData.pmesh->flags[i] = POLYFLAGS_SWIM;
			}
		}
		
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = buildData.pmesh->verts;
		params.vertCount = buildData.pmesh->nverts;
		params.polys = buildData.pmesh->polys;
		params.polyAreas = buildData.pmesh->areas;
		params.polyFlags = buildData.pmesh->flags;
		params.polyCount = buildData.pmesh->npolys;
		params.nvp = buildData.pmesh->nvp;
		params.detailMeshes = buildData.dmesh->meshes;
		params.detailVerts = buildData.dmesh->verts;
		params.detailVertsCount = buildData.dmesh->nverts;
		params.detailTris = buildData.dmesh->tris;
		params.detailTriCount = buildData.dmesh->ntris;
		
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
		rcVcopy(params.bmin, buildData.pmesh->bmin);
		rcVcopy(params.bmax, buildData.pmesh->bmax);
		params.cs = rcConfig.cs;
		params.ch = rcConfig.ch;
		params.buildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &buildData.navData, &navDataSize))
		{
			context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return nullptr;
		}		
	}

	// The NavData is returned, it should not get disposed along with buildData.
	buildData.ownsNavData = false;
	
	dataSize = navDataSize;
	unsigned char* navData = buildData.navData;
	return navData;
}

unsigned char* RecastUnityPluginManager::buildTileMeshWithChunkyMesh(const int tx, const int ty, const NavMeshBuildConfig& config, float tileSize,
	const float* bmin, const float* bmax,
	const NavMeshInputGeometry& inputGeometry, const rcChunkyTriMesh* chunkyMesh, int& dataSize, rcContext& context)
{
	NavMeshBuildData buildData;
	const float* verts = inputGeometry.vertices;
	int nverts = inputGeometry.verticesCount;
	
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

	float tbmin[2], tbmax[2];
	tbmin[0] = rcConfig.bmin[0];
	tbmin[1] = rcConfig.bmin[2];
	tbmax[0] = rcConfig.bmax[0];
	tbmax[1] = rcConfig.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return nullptr;
	}
	
	//
	// Step 2. Rasterize input polygon soup.
	//

	dtStatus status = NavMeshBuildUtility::prepareTriangleRasterization(rcConfig, chunkyMesh->maxTrisPerChunk, buildData, context);
	if (status == DT_FAILURE)
	{
		return nullptr;
	}

	int tileTriCount = 0;
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* ctris = &(chunkyMesh->tris[node.i*3]);
		const int nctris = node.n;
		
		tileTriCount += nctris;
		
		memset(buildData.triareas, 0, nctris*sizeof(unsigned char));
		rcMarkWalkableTriangles(&context, rcConfig.walkableSlopeAngle,
								verts, nverts, ctris, nctris, buildData.triareas);
		
		if (!rcRasterizeTriangles(&context, verts, nverts, ctris, buildData.triareas, nctris, *buildData.solid, rcConfig.walkableClimb))
		{
			return nullptr;
		}
	}

	//
	// Step 3. Filter walkable surfaces.
	//

	NavMeshBuildUtility::filterWalkingSurfaces(config.filterLowHangingObstacles, config.filterLedgeSpans, config.filterWalkableLowHeightSpans,
		rcConfig, buildData, context);

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	if (NavMeshBuildUtility::preparePartionning(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	// TODO: for now there is nothing to mark areas, maybe later.
	// (Optional) Mark areas.
	// const ConvexVolume* vols = m_geom->getConvexVolumes();
	// for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	// 	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	if (NavMeshBuildUtility::buildRegions(config.partitionType, rcConfig.borderSize, rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	//
	// Step 5. Trace and simplify region contours.
	//
	
	if (NavMeshBuildUtility::createContours(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}
	
	if (buildData.cset->nconts == 0)
	{
		return nullptr;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	if (NavMeshBuildUtility::buildMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//
	
	if (NavMeshBuildUtility::buildDetailMesh(rcConfig, buildData, context) == DT_FAILURE)
	{
		return nullptr;
	}

	buildData.navData = 0;
	int navDataSize = 0;
	if (rcConfig.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (buildData.pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			context.log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", buildData.pmesh->nverts, 0xffff);
			return nullptr;
		}
		
		// Update poly flags from areas.
		for (int i = 0; i < buildData.pmesh->npolys; ++i)
		{
			// We have to set a flag different from 0 for the tiles, otherwise it won't work (PassFilter will always return false)
			buildData.pmesh->flags[i] = 1;
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
		params.verts = buildData.pmesh->verts;
		params.vertCount = buildData.pmesh->nverts;
		params.polys = buildData.pmesh->polys;
		params.polyAreas = buildData.pmesh->areas;
		params.polyFlags = buildData.pmesh->flags;
		params.polyCount = buildData.pmesh->npolys;
		params.nvp = buildData.pmesh->nvp;
		params.detailMeshes = buildData.dmesh->meshes;
		params.detailVerts = buildData.dmesh->verts;
		params.detailVertsCount = buildData.dmesh->nverts;
		params.detailTris = buildData.dmesh->tris;
		params.detailTriCount = buildData.dmesh->ntris;
		
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
		rcVcopy(params.bmin, buildData.pmesh->bmin);
		rcVcopy(params.bmax, buildData.pmesh->bmax);
		params.cs = rcConfig.cs;
		params.ch = rcConfig.ch;
		params.buildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &buildData.navData, &navDataSize))
		{
			context.log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return nullptr;
		}		
	}

	// The NavData is returned, it should not get disposed along with buildData.
	buildData.ownsNavData = false;
	
	dataSize = navDataSize;
	unsigned char* navData = buildData.navData;
	return navData;
}

void RecastUnityPluginManager::disposeNavMesh(void* allocatedNavMesh, int environmentId)
{
	if (allocatedNavMesh == nullptr)
	{
		return;
	}
	
	auto navMesh = (dtNavMesh*)allocatedNavMesh;
	// Find the element 
	auto it = std::find_if( 
		s_instance->m_navMeshes.begin(), s_instance->m_navMeshes.end(), [&](const auto& pair) { 
			return pair.first == environmentId 
				   && pair.second == navMesh; 
		}); 
  
	// If found, erase it 
	if (it != s_instance->m_navMeshes.end()) { 
		s_instance->m_navMeshes.erase(it); 
	} 

	dtFreeNavMesh(navMesh);
}


dtStatus RecastUnityPluginManager::createNavMeshQuery(const void* navMesh, int maxNodes, void*& allocatedNavMeshQuery, int environmentId)
{
	auto navMeshQuery = dtAllocNavMeshQuery();
	if (navMeshQuery != nullptr)
	{
		// Store the allocated navmesh query.
		s_instance->m_navMeshQueries.insert({environmentId, navMeshQuery});
		navMeshQuery->init((const dtNavMesh*)navMesh, maxNodes);
		allocatedNavMeshQuery = navMeshQuery;
		return DT_SUCCESS;
	}

	return DT_FAILURE;
}

void RecastUnityPluginManager::disposeNavMeshQuery(void*& allocatedNavMeshQuery, int environmentId)
{
	if (allocatedNavMeshQuery == nullptr)
	{
		return;
	}
	
	auto navMeshQuery = (dtNavMeshQuery*)allocatedNavMeshQuery;
	// Find the element 
	auto it = std::find_if( 
		s_instance->m_navMeshQueries.begin(), s_instance->m_navMeshQueries.end(), [&](const auto& pair) { 
			return pair.first == environmentId 
				   && pair.second == navMeshQuery; 
		}); 
  
	// If found, erase it 
	if (it != s_instance->m_navMeshQueries.end()) { 
		s_instance->m_navMeshQueries.erase(it); 
	} 
	
	dtFreeNavMeshQuery(navMeshQuery);
	allocatedNavMeshQuery = nullptr;
}
