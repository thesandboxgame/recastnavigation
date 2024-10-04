#pragma once

/// A block that has a specific area type (liquid for instance)
/// Used to change the area type of some parts of the navMeshes.
struct BlockArea
{
	/// The world position of the center of the block.
	float center[3];
	/// The type of the block.
	int area;
};
