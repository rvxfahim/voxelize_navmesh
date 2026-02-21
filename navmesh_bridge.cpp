/*
 * navmesh_bridge.cpp — extern "C" shim over Detour dtNavMesh + dtNavMeshQuery.
 *
 * Loads the binary navmesh format saved by RecastDemo (Sample::saveAll):
 *   NavMeshSetHeader  { magic='MSET', version=1, numTiles, dtNavMeshParams }
 *   Per tile:
 *     NavMeshTileHeader { dtTileRef tileRef, int dataSize }
 *     raw tile data (dataSize bytes)
 *
 * All coordinates inside the navmesh are Y-up (Recast convention).
 * Callers that work in Z-up space must swap Y<->Z themselves (see navmesh.py).
 *
 * Build:
 *   cmake --build build --target navmesh_bridge
 */

#include <cstdio>
#include <cstring>
#include <cstdint>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourAlloc.h"

// ---------------------------------------------------------------------------
// Binary format constants (must match RecastDemo/Source/Sample.cpp)
// ---------------------------------------------------------------------------
static const int NAVMESHSET_MAGIC   = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader {
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader {
    dtTileRef tileRef;
    int       dataSize;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline void swap3f(const float* src, float* dst, bool swap_yz) {
    if (swap_yz) {
        dst[0] = src[0];
        dst[1] = src[2];
        dst[2] = src[1];
    } else {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
    }
}

// ---------------------------------------------------------------------------
// Public C API
// ---------------------------------------------------------------------------
extern "C" {

// ---- Navmesh load / free --------------------------------------------------

void* nm_load(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) return nullptr;

    NavMeshSetHeader header;
    if (fread(&header, sizeof(header), 1, f) != 1) { fclose(f); return nullptr; }
    if (header.magic   != NAVMESHSET_MAGIC)   { fclose(f); return nullptr; }
    if (header.version != NAVMESHSET_VERSION) { fclose(f); return nullptr; }

    dtNavMesh* mesh = dtAllocNavMesh();
    if (!mesh) { fclose(f); return nullptr; }

    dtStatus status = mesh->init(&header.params);
    if (dtStatusFailed(status)) { dtFreeNavMesh(mesh); fclose(f); return nullptr; }

    for (int i = 0; i < header.numTiles; ++i) {
        NavMeshTileHeader th;
        if (fread(&th, sizeof(th), 1, f) != 1) break;
        if (!th.tileRef || !th.dataSize)        break;

        unsigned char* data = (unsigned char*)dtAlloc(th.dataSize, DT_ALLOC_PERM);
        if (!data) break;
        memset(data, 0, th.dataSize);

        if (fread(data, th.dataSize, 1, f) != 1) {
            dtFree(data);
            break;
        }
        mesh->addTile(data, th.dataSize, DT_TILE_FREE_DATA, th.tileRef, nullptr);
    }

    fclose(f);
    return mesh;
}

void nm_free(void* handle)
{
    if (handle) dtFreeNavMesh(static_cast<dtNavMesh*>(handle));
}

int nm_tile_count(void* handle)
{
    if (!handle) return 0;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);
    int count = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) ++count;
    }
    return count;
}

// ---- Geometry extraction --------------------------------------------------
// Returns the i-th *valid* tile's detail-mesh vertices as packed floats
// (x,y,z) in Recast Y-up space.  Returns the number of floats written.
// Pass max_floats = 0 to query the required size.

int nm_tile_detail_verts(void* handle, int tile_idx,
                         float* out, int max_floats)
{
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    // Walk valid tiles to find the tile_idx-th one
    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) { tile = t; break; }
        }
    }
    if (!tile) return -1;

    const dtMeshHeader* h = tile->header;
    // Detail verts = poly verts (stored first) + extra detail verts
    int total_verts = h->vertCount + h->detailVertCount;
    int needed = total_verts * 3;
    if (max_floats == 0) return needed;
    if (max_floats < needed) return -1;

    // Poly verts (quantised shorts → world float via tile bbox)
    for (int v = 0; v < h->vertCount; ++v) {
        // tile->verts is already in world-space floats after deserialization
        out[v*3+0] = tile->verts[v*3+0];
        out[v*3+1] = tile->verts[v*3+1];
        out[v*3+2] = tile->verts[v*3+2];
    }
    // Detail extra verts
    for (int v = 0; v < h->detailVertCount; ++v) {
        int base = h->vertCount + v;
        out[base*3+0] = tile->detailVerts[v*3+0];
        out[base*3+1] = tile->detailVerts[v*3+1];
        out[base*3+2] = tile->detailVerts[v*3+2];
    }
    return needed;
}

// Returns the detail triangles for tile tile_idx as packed ints (i0,i1,i2).
// Indices reference the combined vertex array returned by nm_tile_detail_verts.
// Returns the number of ints written, or required size when max_ints==0.

int nm_tile_detail_tris(void* handle, int tile_idx,
                        int* out, int max_ints)
{
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) { tile = t; break; }
        }
    }
    if (!tile) return -1;

    const dtMeshHeader* h = tile->header;
    int needed = h->detailTriCount * 3;
    if (max_ints == 0) return needed;
    if (max_ints < needed) return -1;

    // Each detail triangle: 4 bytes — (v0,v1,v2,flags).
    // Indices < poly->vertCount index into poly->verts (absolute indices into
    // tile->verts); indices >= poly->vertCount are offsets into detailVerts
    // relative to the sub-mesh's vertBase.  We resolve them all to absolute
    // indices in the combined [tile->verts | tile->detailVerts] array.
    int out_idx = 0;
    for (int m = 0; m < h->detailMeshCount; ++m) {
        const dtPolyDetail& dm = tile->detailMeshes[m];
        const dtPoly&       poly = tile->polys[m];

        for (unsigned int t = 0; t < dm.triCount; ++t) {
            const unsigned char* tri =
                &tile->detailTris[(dm.triBase + t) * 4];

            for (int k = 0; k < 3; ++k) {
                unsigned char vi = tri[k];
                int abs_idx;
                if (vi < poly.vertCount) {
                    // Index into tile->verts
                    abs_idx = poly.verts[vi];
                } else {
                    // Index into tile->detailVerts, offset by dm.vertBase
                    abs_idx = h->vertCount + (int)(dm.vertBase + (vi - poly.vertCount));
                }
                out[out_idx++] = abs_idx;
            }
        }
    }
    return out_idx; // == h->detailTriCount * 3 if all went well
}

// Convenience: bounding box of tile tile_idx (Y-up).
// out_min and out_max must each point to a float[3] buffer.
// Returns 0 on success, -1 on failure.
int nm_tile_bounds(void* handle, int tile_idx,
                   float* out_min, float* out_max)
{
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) { tile = t; break; }
        }
    }
    if (!tile) return -1;

    for (int k = 0; k < 3; ++k) {
        out_min[k] = tile->header->bmin[k];
        out_max[k] = tile->header->bmax[k];
    }
    return 0;
}

// ---- NavMeshQuery lifecycle -----------------------------------------------

void* nm_query_create(void* handle, int max_nodes)
{
    if (!handle) return nullptr;
    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    if (!q) return nullptr;
    dtStatus s = q->init(static_cast<dtNavMesh*>(handle), max_nodes);
    if (dtStatusFailed(s)) { dtFreeNavMeshQuery(q); return nullptr; }
    return q;
}

void nm_query_free(void* qhandle)
{
    if (qhandle) dtFreeNavMeshQuery(static_cast<dtNavMeshQuery*>(qhandle));
}

// ---- Navigation queries ---------------------------------------------------
// All positions are in Recast Y-up space (caller swaps before calling).

// Find nearest polygon to pos within search extents.
// Returns the polyref (0 = not found).
// nearest_pt_out must point to float[3]; receives snapped position.

uint32_t nm_find_nearest_poly(void* qhandle,
                               const float* pos,
                               const float* extents,
                               float*       nearest_pt_out)
{
    if (!qhandle) return 0;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);

    dtPolyRef ref = 0;
    float     pt[3] = {0,0,0};
    dtStatus s = q->findNearestPoly(pos, extents, &filter, &ref, pt);
    if (dtStatusFailed(s)) return 0;

    if (nearest_pt_out) {
        nearest_pt_out[0] = pt[0];
        nearest_pt_out[1] = pt[1];
        nearest_pt_out[2] = pt[2];
    }
    return (uint32_t)ref;
}

// Find polygon-corridor path from startRef to endRef.
// poly_path_out: caller-allocated uint32_t[max_path].
// Returns number of polygons in the corridor, or -1 on failure.

int nm_find_path(void* qhandle,
                 uint32_t    start_ref,
                 uint32_t    end_ref,
                 const float* start_pos,
                 const float* end_pos,
                 uint32_t*   poly_path_out,
                 int         max_path)
{
    if (!qhandle || !poly_path_out) return -1;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);

    int npath = 0;
    // dtPolyRef and uint32_t must be same width (both are unsigned int)
    static_assert(sizeof(dtPolyRef) == sizeof(uint32_t), "polyref size mismatch");

    dtStatus s = q->findPath(
        (dtPolyRef)start_ref, (dtPolyRef)end_ref,
        start_pos, end_pos, &filter,
        reinterpret_cast<dtPolyRef*>(poly_path_out), &npath, max_path);

    if (dtStatusFailed(s)) return -1;
    return npath;
}

// Find straight (waypoint) path.
// straight_path_out: caller-allocated float[max_pts * 3].
// Returns number of waypoints, or -1 on failure.

int nm_find_straight_path(void*        qhandle,
                           const float* start_pos,
                           const float* end_pos,
                           const uint32_t* poly_corridor,
                           int          corridor_len,
                           float*       straight_path_out,
                           int          max_pts)
{
    if (!qhandle || !straight_path_out) return -1;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    int nstraight = 0;
    dtStatus s = q->findStraightPath(
        start_pos, end_pos,
        reinterpret_cast<const dtPolyRef*>(poly_corridor), corridor_len,
        straight_path_out,
        nullptr,  // flags (not needed for prototype)
        nullptr,  // refs  (not needed for prototype)
        &nstraight, max_pts);

    if (dtStatusFailed(s)) return -1;
    return nstraight;
}

} // extern "C"
