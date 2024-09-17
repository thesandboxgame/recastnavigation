#pragma once

struct NavMeshBuildConfig
{
	/// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] 
	float cs;

	/// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	float ch;

	/// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] 
	float walkableSlopeAngle;

	/// The height (m) of the agents that will use the NavMesh to compute their path.
	float agentHeight;

	/// The max height (m) that the agents can climb.
	float agentMaxClimb;

	/// The radius (m) of the agents that will use the NavMesh to compute their path.
	float agentRadius;

	/// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] 
	int maxEdgeLen;

	/// The maximum distance a simplified contour's border edges should deviate 
	/// the original raw contour. [Limit: >=0] [Units: vx]
	float maxSimplificationError;

	/// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] 
	int minRegionArea;

	/// Any regions with a span count smaller than this value will, if possible, 
	/// be merged with larger regions. [Limit: >=0] [Units: vx] 
	int mergeRegionArea;

	/// The maximum number of vertices allowed for polygons generated during the 
	/// contour to polygon conversion process. [Limit: >= 3] 
	int maxVertsPerPoly;

	/// Sets the sampling distance to use when generating the detail mesh.
	/// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu] 
	float detailSampleDist;

	/// The maximum distance the detail mesh surface should deviate from heightfield
	/// data. (For height detail only.) [Limit: >=0] [Units: wu] 
	float detailSampleMaxError;

	/// Marks non-walkable spans as walkable if their maximum is within walkableClimb of the span below them.
	/// This removes small obstacles and rasterization artifacts that the agent would be able to walk over such as curbs. It also allows agents to move up terraced structures like stairs.
	bool filterLowHangingObstacles;

	/// Marks spans that are ledges as not-walkable.
	/// A ledge is a span with one or more neighbors whose maximum is further away than walkableClimb from the current span's maximum.
	/// This removes the impact of the overestimation of conservative voxelization so the resulting mesh will not have regions hanging in the air over ledges
	bool filterLedgeSpans;

	/// Marks walkable spans as not walkable if the clearance above the span is less than the specified walkableHeight.
	bool filterWalkableLowHeightSpans;

	/// The type of NavMesh partitioning (see PartitionType enum)
	int partitionType;
};
