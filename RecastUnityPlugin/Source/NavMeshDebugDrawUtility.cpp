#include "NavMeshDebugDrawUtility.h"

bool NavMeshDebugDrawUtility::fetchTileNavMeshDebugDrawData(const dtNavMesh& mesh, NavMeshDebugDrawData* debugDrawData)
{
	bool couldBufferContainAllData = true;
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		if (!fetchMeshTileData(mesh, tile, debugDrawData))
		{
			couldBufferContainAllData = false;
		}
	}

	return couldBufferContainAllData;
}

bool NavMeshDebugDrawUtility::fetchMeshTileData(const dtNavMesh& /*mesh*/, const dtMeshTile* tile, NavMeshDebugDrawData* debugDrawData)
{
	bool isDataBufferFull = false;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
			continue;
			
		const dtPolyDetail* pd = &tile->detailMeshes[i];
		
		unsigned char polyArea = p->getArea();
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
			const float* trianglesPositions[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->vertCount)
					trianglesPositions[k] = &tile->verts[p->verts[t[k]]*3];
				else
					trianglesPositions[k] = &tile->detailVerts[(pd->vertBase+t[k]-p->vertCount)*3];
			}

			isDataBufferFull |= !debugDrawData->addPolyTriangle(trianglesPositions, polyArea);
		}
	}

	isDataBufferFull |= !fetchMeshBoundariesData(tile, debugDrawData);
	
	return !isDataBufferFull;
}

bool NavMeshDebugDrawUtility::fetchMeshBoundariesData(const dtMeshTile* tile, NavMeshDebugDrawData* debugDrawData)
{
	static const float thr = 0.01f*0.01f;
	bool isDataBufferFull = false;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
		
		const dtPolyDetail* pd = &tile->detailMeshes[i];
		
		for (int j = 0, nj = (int)p->vertCount; j < nj; ++j)
		{
			bool inner = p->neis[j] != 0;
			BoundaryType boundaryType = inner ? INNER : OUTER;
			if (inner)
			{
				if (p->neis[j] & DT_EXT_LINK)
				{
					bool con = false;
					for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
					{
						if (tile->links[k].edge == j)
						{
							con = true;
							break;
						}
					}
					if (con)
						boundaryType = TILE;
					else
						boundaryType = UNCONNECTED_EDGE;
				}
			}
			
			const float* v0 = &tile->verts[p->verts[j]*3];
			const float* v1 = &tile->verts[p->verts[(j+1) % nj]*3];
			
			// Draw detail mesh edges which align with the actual poly edge.
			// This is really slow.
			for (int k = 0; k < pd->triCount; ++k)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase+k)*4];
				const float* tv[3];
				for (int m = 0; m < 3; ++m)
				{
					if (t[m] < p->vertCount)
						tv[m] = &tile->verts[p->verts[t[m]]*3];
					else
						tv[m] = &tile->detailVerts[(pd->vertBase+(t[m]-p->vertCount))*3];
				}
				for (int m = 0, n = 2; m < 3; n=m++)
				{
					if ((dtGetDetailTriEdgeFlags(t[3], n) & DT_DETAIL_EDGE_BOUNDARY) == 0)
						continue;

					if (distancePtLine2d(tv[n],v0,v1) < thr &&
						distancePtLine2d(tv[m],v0,v1) < thr)
					{
						const float* linePositions[2];
						linePositions[0] = tv[n];
						linePositions[1] = tv[m];
						isDataBufferFull |= !debugDrawData->addBoundary(linePositions, boundaryType);
					}
				}
			}
		}
	}

	return !isDataBufferFull;
}

float NavMeshDebugDrawUtility::distancePtLine2d(const float* pt, const float* p, const float* q)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d != 0) t /= d;
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}