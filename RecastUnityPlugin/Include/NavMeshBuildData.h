#pragma once
#include "ChunkyTriMesh.h"
#include "Recast.h"

/// Contains the intermediate data used when building a NavMesh.
/// Simplifies the memory freeing.
struct NavMeshBuildData
{
	NavMeshBuildData()
		: solid(nullptr),
		  triareas(nullptr),
		  chf(nullptr),
		  cset(nullptr),
		  pmesh(nullptr),
		  dmesh(nullptr),
		  navData(nullptr),
		  chunkyMesh(nullptr)
	{
	}

	~NavMeshBuildData()
	{
		if (solid != nullptr)
		{
			rcFreeHeightField(solid);
			solid = nullptr;
		}

		if (triareas != nullptr)
		{
			delete [] triareas;
			triareas = nullptr;
		}

		if (chf != nullptr)
		{
			rcFreeCompactHeightfield(chf);
			chf = nullptr;
		}

		if (cset != nullptr)
		{
			rcFreeContourSet(cset);
			cset = nullptr;
		}

		if (pmesh != nullptr)
		{
			rcFreePolyMesh(pmesh);
			pmesh = nullptr;
		}

		if (dmesh != nullptr)
		{
			rcFreePolyMeshDetail(dmesh);
			dmesh = nullptr;
		}

		if (disposeNavData && navData != nullptr)
		{
			dtFree(navData);
			navData = nullptr;
		}
	}

	rcHeightfield* solid;
	unsigned char* triareas;
	rcCompactHeightfield* chf;
	rcContourSet* cset;
	rcPolyMesh* pmesh;
	rcPolyMeshDetail* dmesh;
	unsigned char* navData;
	rcChunkyTriMesh* chunkyMesh;
	bool disposeNavData;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	NavMeshBuildData(const NavMeshBuildData&);
	NavMeshBuildData& operator=(const NavMeshBuildData&);
};
