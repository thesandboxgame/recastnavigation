#pragma once
#include "DetourStatus.h"
#include "NavMeshBuildData.h"

class NavMeshBuildUtility
{
public:
	static dtStatus prepareTriangleRasterization(const rcConfig& rcConfig, int triAreasCount,
	                                             NavMeshBuildData& buildData, rcContext& context);

	static void filterWalkingSurfaces(bool filterLowHangingObstacles, bool filterLedgeSpans,
	                                  bool filterWalkableLowHeightSpans,
	                                  const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);

	static dtStatus preparePartionning(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);

	static dtStatus buildRegions(int partitionType, int borderSize,
	                             const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);

	static dtStatus createContours(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);

	static dtStatus buildMesh(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);

	static dtStatus buildDetailMesh(const rcConfig& rcConfig, NavMeshBuildData& buildData, rcContext& context);
};
