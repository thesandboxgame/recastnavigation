#include "NavMeshBuildUtility.h"

#include "DetourNavMeshBuilder.h"
#include "RecastUnityPluginManager.h"

dtStatus NavMeshBuildUtility::prepareTriangleRasterization(const rcConfig& rcConfig, int triAreasCount, NavMeshBuildData& buildData, rcContext& context)
{
	// Allocate voxel heightfield where we rasterize our input data to.
	buildData.solid = rcAllocHeightfield();
	if (!buildData.solid)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return DT_FAILURE;
	}
	if (!rcCreateHeightfield(&context, *buildData.solid, rcConfig.width, rcConfig.height, rcConfig.bmin, rcConfig.bmax, rcConfig.cs, rcConfig.ch))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return DT_FAILURE;
	}
	
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	buildData.triareas = new unsigned char[triAreasCount];
	if (!buildData.triareas)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", triAreasCount);
		return DT_FAILURE;
	}

	return DT_SUCCESS;
}

void NavMeshBuildUtility::filterWalkingSurfaces(bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans,
	const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(&context, rcConfig.walkableClimb, *buildData.solid);
	if (filterLedgeSpans)
		rcFilterLedgeSpans(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *buildData.solid);
	if (filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(&context, rcConfig.walkableHeight, *buildData.solid);
}

dtStatus NavMeshBuildUtility::preparePartionning(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	buildData.chf = rcAllocCompactHeightfield();
	if (!buildData.chf )
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return DT_FAILURE;
	}
	if (!rcBuildCompactHeightfield(&context, rcConfig.walkableHeight, rcConfig.walkableClimb, *buildData.solid, *buildData.chf ))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return DT_FAILURE;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(&context, rcConfig.walkableRadius, *buildData.chf ))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return DT_FAILURE;
	}

	return DT_SUCCESS;
}

dtStatus NavMeshBuildUtility::buildRegions(int partitionType, int borderSize, const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
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

	if (partitionType == PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(&context, *buildData.chf))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return DT_FAILURE;
		}
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(&context, *buildData.chf, borderSize, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return DT_FAILURE;
		}
	}
	else if (partitionType == PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(&context, *buildData.chf, borderSize, rcConfig.minRegionArea, rcConfig.mergeRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return DT_FAILURE;
		}
	}
	else // PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(&context, *buildData.chf, borderSize, rcConfig.minRegionArea))
		{
			context.log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return DT_FAILURE;
		}
	}

	return DT_SUCCESS;
}

dtStatus NavMeshBuildUtility::createContours(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
	// Create contours.
	buildData.cset = rcAllocContourSet();
	if (!buildData.cset)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return DT_FAILURE;
	}
	if (!rcBuildContours(&context, *buildData.chf, rcConfig.maxSimplificationError, rcConfig.maxEdgeLen, *buildData.cset))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return DT_FAILURE;
	}
	
	return DT_SUCCESS;
}

dtStatus NavMeshBuildUtility::buildMesh(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
	// Build polygon navmesh from the contours.
	buildData.pmesh = rcAllocPolyMesh();
	if (!buildData.pmesh)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return DT_FAILURE;
	}
	if (!rcBuildPolyMesh(&context, *buildData.cset, rcConfig.maxVertsPerPoly, *buildData.pmesh))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return DT_FAILURE;
	}

	return DT_SUCCESS;
}

dtStatus NavMeshBuildUtility::buildDetailMesh(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context)
{
	buildData.dmesh = rcAllocPolyMeshDetail();
	if (!buildData.dmesh)
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return DT_FAILURE;
	}

	if (!rcBuildPolyMeshDetail(&context, *buildData.pmesh, *buildData.chf, rcConfig.detailSampleDist, rcConfig.detailSampleMaxError, *buildData.dmesh))
	{
		context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return DT_FAILURE;
	}

	return DT_SUCCESS;
}