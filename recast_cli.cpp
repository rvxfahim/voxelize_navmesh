/*
 * recast_cli.cpp — Minimal CLI that loads an OBJ, builds a Recast navmesh,
 * runs Detour pathfinding, and prints JSON to stdout.
 *
 * Usage:
 *   ./recast_cli <obj_file> <sx> <sy> <sz> <ex> <ey> <ez> [--save-bin <out.bin>]
 *
 * Output (JSON):
 *   { "status": "ok", "path": [[x,y,z], ...], "navmesh": { "verts": N, "polys": N } }
 *   or
 *   { "status": "error", "message": "..." }
 *
 * --save-bin writes the baked dtNavMesh in the exact binary format used by
 * RecastDemo's Save button (Sample::saveAll), so navmesh_bridge / navmesh.py
 * can load it directly.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

// --------------------------------------------------------------------------
// Minimal OBJ loader (vertices + triangulated faces only)
// --------------------------------------------------------------------------
struct ObjMesh {
    std::vector<float> verts;   // flat: x,y,z,x,y,z,...
    std::vector<int>   tris;    // flat: i0,i1,i2,...
    int numVerts() const { return (int)verts.size() / 3; }
    int numTris()  const { return (int)tris.size() / 3; }
};

static bool loadObj(const char* path, ObjMesh& mesh) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 2) continue;
        if (line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            if (sscanf(line.c_str() + 2, "%f %f %f", &x, &y, &z) == 3) {
                mesh.verts.push_back(x);
                mesh.verts.push_back(y);
                mesh.verts.push_back(z);
            }
        } else if (line[0] == 'f' && line[1] == ' ') {
            // Parse face — handles "f v", "f v/vt", "f v/vt/vn", "f v//vn"
            std::istringstream ss(line.substr(2));
            std::vector<int> faceVerts;
            std::string token;
            while (ss >> token) {
                int vi = std::atoi(token.c_str());  // stops at '/'
                if (vi < 0) vi = mesh.numVerts() + vi + 1;  // relative index
                faceVerts.push_back(vi - 1);  // OBJ is 1-indexed
            }
            // Triangulate fan
            for (int i = 1; i + 1 < (int)faceVerts.size(); i++) {
                mesh.tris.push_back(faceVerts[0]);
                mesh.tris.push_back(faceVerts[i]);
                mesh.tris.push_back(faceVerts[i + 1]);
            }
        }
    }
    return mesh.numVerts() > 0 && mesh.numTris() > 0;
}

// --------------------------------------------------------------------------
// Binary navmesh save (same format as RecastDemo Sample::saveAll)
// --------------------------------------------------------------------------
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

static bool saveNavMeshBin(const char* path, const dtNavMesh* mesh) {
    FILE* f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "saveNavMeshBin: cannot open '%s' for writing\n", path);
        return false;
    }

    NavMeshSetHeader header;
    header.magic   = NAVMESHSET_MAGIC;
    header.version = NAVMESHSET_VERSION;
    header.numTiles = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) ++header.numTiles;
    }
    memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
    fwrite(&header, sizeof(header), 1, f);

    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (!t || !t->header || !t->dataSize) continue;

        NavMeshTileHeader th;
        th.tileRef  = mesh->getTileRef(t);
        th.dataSize = t->dataSize;
        fwrite(&th, sizeof(th), 1, f);
        fwrite(t->data, t->dataSize, 1, f);
    }

    fclose(f);
    return true;
}

// --------------------------------------------------------------------------
// JSON helpers
// --------------------------------------------------------------------------
static void jsonError(const char* msg) {
    printf("{\"status\":\"error\",\"message\":\"%s\"}\n", msg);
}

// --------------------------------------------------------------------------
// Recast context — use base class directly (log goes to stderr by default)
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Main
// --------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 8) {
        fprintf(stderr, "Usage: %s <obj> <sx> <sy> <sz> <ex> <ey> <ez> [--save-bin <out.bin>]\n", argv[0]);
        jsonError("invalid arguments");
        return 1;
    }

    const char* objPath = argv[1];
    float startPos[3] = { (float)atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]) };
    float endPos[3]   = { (float)atof(argv[5]), (float)atof(argv[6]), (float)atof(argv[7]) };

    // Optional: --save-bin <path>
    const char* saveBinPath = nullptr;
    for (int i = 8; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--save-bin") {
            saveBinPath = argv[i + 1];
            break;
        }
    }

    // --- Load OBJ ---
    ObjMesh obj;
    if (!loadObj(objPath, obj)) {
        jsonError("failed to load OBJ file");
        return 1;
    }
    fprintf(stderr, "Loaded OBJ: %d verts, %d tris\n", obj.numVerts(), obj.numTris());

    // --- Compute bounds ---
    float bmin[3], bmax[3];
    rcCalcBounds(obj.verts.data(), obj.numVerts(), bmin, bmax);
    fprintf(stderr, "Bounds: [%.2f,%.2f,%.2f] - [%.2f,%.2f,%.2f]\n",
            bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2]);

    // --- Configure Recast ---
    float extent[3] = { bmax[0]-bmin[0], bmax[1]-bmin[1], bmax[2]-bmin[2] };
    float maxExtent = std::fmax(extent[0], std::fmax(extent[1], extent[2]));

    // Use a coarser cell size than the voxel mesh to avoid exact boundary
    // alignment (voxel mesh was built at maxExtent/200). A ~3x coarser grid
    // still gives good navmesh quality while keeping spans manageable.
    float cellSize = maxExtent / 80.0f;
    float cellHeight = cellSize * 0.5f;  // finer vertical resolution

    // Pad bounds slightly to avoid edge cases with axis-aligned geometry
    float pad = cellSize * 2.0f;
    bmin[0] -= pad; bmin[1] -= pad; bmin[2] -= pad;
    bmax[0] += pad; bmax[1] += pad; bmax[2] += pad;

    // Agent dimensions in world units
    float agentHeight = 1.8f;
    float agentRadius = 0.3f;
    float agentMaxClimb = 0.5f;

    rcConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cs  = cellSize;
    cfg.ch  = cellHeight;
    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);
    rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

    cfg.walkableSlopeAngle  = 45.0f;
    cfg.walkableHeight      = (int)ceilf(agentHeight / cfg.ch);
    cfg.walkableClimb       = (int)floorf(agentMaxClimb / cfg.ch);
    cfg.walkableRadius      = (int)ceilf(agentRadius / cfg.cs);
    cfg.maxEdgeLen          = (int)(12.0f / cfg.cs);
    cfg.maxSimplificationError = 1.3f;
    cfg.minRegionArea       = 8;
    cfg.mergeRegionArea     = 20;
    cfg.maxVertsPerPoly     = 6;
    cfg.detailSampleDist    = cfg.cs * 6.0f;
    cfg.detailSampleMaxError = cfg.ch * 1.0f;

    if (cfg.detailSampleDist < 0.9f)
        cfg.detailSampleDist = 0.0f;

    fprintf(stderr, "Grid: %d x %d, cs=%.4f ch=%.4f\n", cfg.width, cfg.height, cfg.cs, cfg.ch);

    rcContext ctx;
    ctx.enableLog(true);

    // --- Rasterize ---
    rcHeightfield* hf = rcAllocHeightfield();
    if (!hf || !rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height,
                                     cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
        jsonError("failed to create heightfield");
        return 1;
    }

    int ntris = obj.numTris();
    fprintf(stderr, "Marking walkable triangles (%d tris) ...\n", ntris);
    unsigned char* triAreas = new unsigned char[ntris]();
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle,
                            obj.verts.data(), obj.numVerts(),
                            obj.tris.data(), ntris, triAreas);
    fprintf(stderr, "Rasterizing ...\n");
    if (!rcRasterizeTriangles(&ctx, obj.verts.data(), obj.numVerts(),
                              obj.tris.data(), triAreas, ntris,
                              *hf, cfg.walkableClimb)) {
        jsonError("failed to rasterize triangles");
        return 1;
    }
    delete[] triAreas;

    fprintf(stderr, "Filtering ...\n");
    // --- Filter walkable surfaces ---
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);

    int spanCount = rcGetHeightFieldSpanCount(&ctx, *hf);
    fprintf(stderr, "Heightfield spans: %d  (walkableHeight=%d, walkableClimb=%d)\n",
            spanCount, cfg.walkableHeight, cfg.walkableClimb);
    if (spanCount > 10000000) {
        jsonError("heightfield has too many spans — cell size may be too small");
        return 1;
    }

    // --- Build compact heightfield ---
    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!chf || !rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf)) {
        jsonError("failed to build compact heightfield");
        return 1;
    }
    rcFreeHeightField(hf);

    // --- Erode + regions ---
    if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf)) {
        jsonError("failed to erode walkable area");
        return 1;
    }
    if (!rcBuildDistanceField(&ctx, *chf)) {
        jsonError("failed to build distance field");
        return 1;
    }
    if (!rcBuildRegions(&ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) {
        jsonError("failed to build regions");
        return 1;
    }

    // --- Contours ---
    rcContourSet* cset = rcAllocContourSet();
    if (!cset || !rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset)) {
        jsonError("failed to build contours");
        return 1;
    }

    // --- Poly mesh ---
    rcPolyMesh* pmesh = rcAllocPolyMesh();
    if (!pmesh || !rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *pmesh)) {
        jsonError("failed to build poly mesh");
        return 1;
    }

    // --- Detail mesh ---
    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    if (!dmesh || !rcBuildPolyMeshDetail(&ctx, *pmesh, *chf, cfg.detailSampleDist,
                                          cfg.detailSampleMaxError, *dmesh)) {
        jsonError("failed to build detail mesh");
        return 1;
    }
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);

    fprintf(stderr, "Navmesh: %d verts, %d polys\n", pmesh->nverts, pmesh->npolys);

    // --- Set walkable flags (required for Detour query filter) ---
    for (int i = 0; i < pmesh->npolys; i++) {
        if (pmesh->areas[i] == RC_WALKABLE_AREA)
            pmesh->flags[i] = 1;
    }

    // --- Build Detour navmesh data ---
    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts            = pmesh->verts;
    params.vertCount        = pmesh->nverts;
    params.polys            = pmesh->polys;
    params.polyFlags        = pmesh->flags;
    params.polyAreas        = pmesh->areas;
    params.polyCount        = pmesh->npolys;
    params.nvp              = pmesh->nvp;
    params.detailMeshes     = dmesh->meshes;
    params.detailVerts      = dmesh->verts;
    params.detailVertsCount = dmesh->nverts;
    params.detailTris       = dmesh->tris;
    params.detailTriCount   = dmesh->ntris;
    params.offMeshConCount  = 0;
    rcVcopy(params.bmin, pmesh->bmin);
    rcVcopy(params.bmax, pmesh->bmax);
    params.walkableHeight   = (float)cfg.walkableHeight * cfg.ch;
    params.walkableRadius   = (float)cfg.walkableRadius * cfg.cs;
    params.walkableClimb    = (float)cfg.walkableClimb  * cfg.ch;
    params.cs               = cfg.cs;
    params.ch               = cfg.ch;
    params.buildBvTree      = true;

    unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
        jsonError("failed to create Detour navmesh data");
        return 1;
    }

    // --- Init Detour navmesh ---
    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        jsonError("failed to alloc navmesh");
        dtFree(navData);
        return 1;
    }
    dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) {
        jsonError("failed to init navmesh");
        dtFreeNavMesh(navMesh);
        return 1;
    }

    // --- Optionally save .bin (RecastDemo format) ---
    if (saveBinPath) {
        if (saveNavMeshBin(saveBinPath, navMesh))
            fprintf(stderr, "Saved navmesh to '%s'\n", saveBinPath);
        else
            fprintf(stderr, "WARNING: failed to save navmesh to '%s'\n", saveBinPath);
    }

    // --- Init query ---
    dtNavMeshQuery* query = dtAllocNavMeshQuery();
    if (!query) {
        jsonError("failed to alloc query");
        dtFreeNavMesh(navMesh);
        return 1;
    }
    status = query->init(navMesh, 2048);
    if (dtStatusFailed(status)) {
        jsonError("failed to init query");
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    // --- Find path ---
    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);

    // Use generous search extents to snap to nearest poly
    float halfExtents[3];
    halfExtents[0] = maxExtent * 0.1f;
    halfExtents[1] = maxExtent * 0.2f;
    halfExtents[2] = maxExtent * 0.1f;

    dtPolyRef startRef = 0, endRef = 0;
    float startPt[3], endPt[3];
    status = query->findNearestPoly(startPos, halfExtents, &filter, &startRef, startPt);
    if (dtStatusFailed(status) || startRef == 0) {
        jsonError("start point not on navmesh");
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    status = query->findNearestPoly(endPos, halfExtents, &filter, &endRef, endPt);
    if (dtStatusFailed(status) || endRef == 0) {
        jsonError("end point not on navmesh");
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    fprintf(stderr, "Start snapped: [%.3f, %.3f, %.3f] ref=%u\n", startPt[0], startPt[1], startPt[2], startRef);
    fprintf(stderr, "End   snapped: [%.3f, %.3f, %.3f] ref=%u\n", endPt[0], endPt[1], endPt[2], endRef);

    // Polygon corridor
    static const int MAX_POLYS = 2048;
    dtPolyRef polyCorridor[MAX_POLYS];
    int npath = 0;
    status = query->findPath(startRef, endRef, startPt, endPt, &filter,
                             polyCorridor, &npath, MAX_POLYS);
    if (dtStatusFailed(status) || npath == 0) {
        jsonError("no path found");
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    // Straight path (actual waypoints)
    static const int MAX_STRAIGHT = 2048;
    float straightPath[MAX_STRAIGHT * 3];
    unsigned char straightPathFlags[MAX_STRAIGHT];
    dtPolyRef straightPathRefs[MAX_STRAIGHT];
    int nstraight = 0;
    status = query->findStraightPath(startPt, endPt, polyCorridor, npath,
                                     straightPath, straightPathFlags, straightPathRefs,
                                     &nstraight, MAX_STRAIGHT);
    if (dtStatusFailed(status) || nstraight == 0) {
        jsonError("failed to compute straight path");
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    fprintf(stderr, "Path: %d corridor polys, %d straight waypoints\n", npath, nstraight);

    // --- Output JSON ---
    printf("{\"status\":\"ok\",\"navmesh\":{\"verts\":%d,\"polys\":%d},\"path\":[",
           pmesh->nverts, pmesh->npolys);
    for (int i = 0; i < nstraight; i++) {
        if (i > 0) printf(",");
        printf("[%.6f,%.6f,%.6f]",
               straightPath[i*3+0], straightPath[i*3+1], straightPath[i*3+2]);
    }
    printf("]}\n");

    // --- Cleanup ---
    dtFreeNavMeshQuery(query);
    dtFreeNavMesh(navMesh);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);

    return 0;
}
