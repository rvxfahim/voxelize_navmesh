/*
 * navmesh_bridge.cpp — C shim over Recast/Detour for loading/query/crowd
 * plus solo-mesh baking from OBJ.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "Recast.h"
#include "fastlz.h"

// Minimal context implementation for Recast that doesn't use virtual inheritance
// to avoid RTTI issues with static library linkage
struct MinimalRecastContext
{
    rcContext ctx;
    
    MinimalRecastContext() : ctx(false) {}  // Disable logging/timers to avoid v-table issues
};


static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader {
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader {
    dtTileRef tileRef;
    int dataSize;
};

struct ObjMesh {
    std::vector<float> verts;
    std::vector<int> tris;
};

static void set_error(char* error_out, int error_out_len, const char* message) {
    if (!error_out || error_out_len <= 0) {
        return;
    }
    std::snprintf(error_out, static_cast<size_t>(error_out_len), "%s", message ? message : "Unknown error");
}

static int parse_face_token(const std::string& token, int vertex_count) {
    if (token.empty()) {
        return -1;
    }
    size_t slash = token.find('/');
    std::string head = slash == std::string::npos ? token : token.substr(0, slash);
    if (head.empty()) {
        return -1;
    }
    int index = std::atoi(head.c_str());
    if (index > 0) {
        index -= 1;
    } else if (index < 0) {
        index = vertex_count + index;
    } else {
        return -1;
    }
    if (index < 0 || index >= vertex_count) {
        return -1;
    }
    return index;
}

static bool load_obj_mesh(const char* obj_path, bool input_is_z_up, ObjMesh& out_mesh, char* error_out, int error_out_len) {
    std::ifstream in(obj_path);
    if (!in.is_open()) {
        set_error(error_out, error_out_len, "Could not open OBJ file.");
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (line.size() > 1 && line[0] == 'v' && std::isspace(static_cast<unsigned char>(line[1]))) {
            std::istringstream iss(line.substr(1));
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (!(iss >> x >> y >> z)) {
                continue;
            }
            if (input_is_z_up) {
                std::swap(y, z);
            }
            out_mesh.verts.push_back(x);
            out_mesh.verts.push_back(y);
            out_mesh.verts.push_back(z);
            continue;
        }
        if (line.size() > 1 && line[0] == 'f' && std::isspace(static_cast<unsigned char>(line[1]))) {
            std::istringstream iss(line.substr(1));
            std::vector<int> face;
            std::string token;
            const int vertex_count = static_cast<int>(out_mesh.verts.size() / 3);
            while (iss >> token) {
                int idx = parse_face_token(token, vertex_count);
                if (idx >= 0) {
                    face.push_back(idx);
                }
            }
            if (face.size() < 3) {
                continue;
            }
            if (input_is_z_up) {
                std::reverse(face.begin(), face.end());
            }
            for (size_t i = 2; i < face.size(); ++i) {
                out_mesh.tris.push_back(face[0]);
                out_mesh.tris.push_back(face[i - 1]);
                out_mesh.tris.push_back(face[i]);
            }
        }
    }

    if (out_mesh.verts.empty() || out_mesh.tris.empty()) {
        set_error(error_out, error_out_len, "OBJ mesh has no valid vertices/faces.");
        return false;
    }

    return true;
}

static void calc_bounds(const std::vector<float>& verts, float out_min[3], float out_max[3]) {
    if (verts.empty()) {
        out_min[0] = out_min[1] = out_min[2] = 0.0f;
        out_max[0] = out_max[1] = out_max[2] = 0.0f;
        return;
    }
    
    out_min[0] = out_max[0] = verts[0];
    out_min[1] = out_max[1] = verts[1];
    out_min[2] = out_max[2] = verts[2];

    const int n = static_cast<int>(verts.size() / 3);
    for (int i = 1; i < n; ++i) {
        const float* v = &verts[i * 3];
        out_min[0] = std::min(out_min[0], v[0]);
        out_min[1] = std::min(out_min[1], v[1]);
        out_min[2] = std::min(out_min[2], v[2]);
        out_max[0] = std::max(out_max[0], v[0]);
        out_max[1] = std::max(out_max[1], v[1]);
        out_max[2] = std::max(out_max[2], v[2]);
    }
}


// ─── TileCache helpers ────────────────────────────────────────────────────────

namespace {

struct TCFastLZCompressor : dtTileCacheCompressor
{
    ~TCFastLZCompressor() override = default;

    int maxCompressedSize(const int bufferSize) override {
        return static_cast<int>(static_cast<float>(bufferSize) * 1.05f);
    }

    dtStatus compress(const unsigned char* buffer, const int bufferSize,
                      unsigned char* compressed, const int, int* compressedSize) override {
        *compressedSize = fastlz_compress(buffer, bufferSize, compressed);
        return DT_SUCCESS;
    }

    dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                        unsigned char* buffer, const int maxBufferSize, int* bufferSize) override {
        *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
        return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
    }
};

struct TCLinearAllocator : dtTileCacheAlloc
{
    unsigned char* buf  = nullptr;
    size_t cap  = 0;
    size_t top  = 0;
    size_t high = 0;

    explicit TCLinearAllocator(size_t capacity) {
        buf = static_cast<unsigned char*>(dtAlloc(capacity, DT_ALLOC_PERM));
        cap = capacity;
    }
    ~TCLinearAllocator() override { dtFree(buf); }

    void reset() override {
        high = dtMax(high, top);
        top = 0;
    }
    void* alloc(size_t size) override {
        if (!buf || top + size > cap) return nullptr;
        unsigned char* mem = &buf[top];
        top += size;
        return mem;
    }
    void free(void*) override {}
};

struct TCSimpleMeshProcess : dtTileCacheMeshProcess
{
    ~TCSimpleMeshProcess() override = default;

    void process(dtNavMeshCreateParams* params,
                 unsigned char* polyAreas,
                 unsigned short* polyFlags) override {
        for (int i = 0; i < params->polyCount; ++i) {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
                polyAreas[i] = 0;
            polyFlags[i] = (polyAreas[i] == 0) ? 1 : 0;
        }
    }
};

} // anonymous namespace

struct TileCacheHandle {
    dtTileCache*         tc       = nullptr;
    dtNavMesh*           nav      = nullptr;
    TCLinearAllocator*   alloc    = nullptr;
    TCFastLZCompressor*  comp     = nullptr;
    TCSimpleMeshProcess* meshproc = nullptr;
};

static const int TILECACHESET_MAGIC   = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static const int TILECACHESET_VERSION = 1;

struct TileCacheSetHeader {
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams   meshParams;
    dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader {
    dtCompressedTileRef tileRef;
    int dataSize;
};

struct TileLayerData {
    unsigned char* data     = nullptr;
    int            dataSize = 0;
};

static const int TC_MAX_LAYERS = 32;

// Rasterize one tile (tx, ty) from the flat mesh into compressed layer blobs.
// base_cfg must already have tileSize, borderSize, width, height (per-tile dims) set.
// Returns the number of layers produced; fills out_tiles[0..return-1].
// The caller owns out_tiles[i].data (dtFree unless handed to addTile).
static int rasterize_tile_layers(
    rcContext*             ctx,
    const float*           verts, int nverts,
    const int*             tris,  int ntris,
    const rcConfig&        base_cfg,
    bool                   filter_low,
    bool                   filter_ledge,
    bool                   filter_low_height,
    int                    tile_x,
    int                    tile_y,
    TileLayerData*         out_tiles,
    int                    max_tiles
) {
    TCFastLZCompressor comp;

    const float tcs = base_cfg.tileSize * base_cfg.cs;

    rcConfig tcfg;
    std::memcpy(&tcfg, &base_cfg, sizeof(rcConfig));
    tcfg.bmin[0] = base_cfg.bmin[0] + tile_x * tcs;
    tcfg.bmin[1] = base_cfg.bmin[1];
    tcfg.bmin[2] = base_cfg.bmin[2] + tile_y * tcs;
    tcfg.bmax[0] = base_cfg.bmin[0] + (tile_x + 1) * tcs;
    tcfg.bmax[1] = base_cfg.bmax[1];
    tcfg.bmax[2] = base_cfg.bmin[2] + (tile_y + 1) * tcs;
    // Expand by border
    tcfg.bmin[0] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmin[2] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmax[0] += static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmax[2] += static_cast<float>(tcfg.borderSize) * tcfg.cs;

    rcHeightfield* solid = rcAllocHeightfield();
    if (!solid) return 0;

    if (!rcCreateHeightfield(ctx, *solid, tcfg.width, tcfg.height,
                              tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch)) {
        rcFreeHeightField(solid);
        return 0;
    }

    std::vector<unsigned char> tri_areas(ntris, 0);
    rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle, verts, nverts, tris, ntris, tri_areas.data());

    if (!rcRasterizeTriangles(ctx, verts, nverts, tris, tri_areas.data(), ntris,
                               *solid, tcfg.walkableClimb)) {
        rcFreeHeightField(solid);
        return 0;
    }

    if (filter_low)        rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *solid);
    if (filter_ledge)      rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *solid);
    if (filter_low_height) rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *solid);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!chf) { rcFreeHeightField(solid); return 0; }
    if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *solid, *chf)) {
        rcFreeHeightField(solid); rcFreeCompactHeightfield(chf); return 0;
    }
    rcFreeHeightField(solid);

    if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *chf)) {
        rcFreeCompactHeightfield(chf); return 0;
    }

    rcHeightfieldLayerSet* lset = rcAllocHeightfieldLayerSet();
    if (!lset) { rcFreeCompactHeightfield(chf); return 0; }
    if (!rcBuildHeightfieldLayers(ctx, *chf, tcfg.borderSize, tcfg.walkableHeight, *lset)) {
        rcFreeCompactHeightfield(chf); rcFreeHeightfieldLayerSet(lset); return 0;
    }
    rcFreeCompactHeightfield(chf);

    int ntiles_out = 0;
    for (int i = 0; i < rcMin(lset->nlayers, max_tiles); ++i) {
        const rcHeightfieldLayer* layer = &lset->layers[i];

        dtTileCacheLayerHeader header{};
        header.magic   = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;
        header.tx      = tile_x;
        header.ty      = tile_y;
        header.tlayer  = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);
        header.width  = static_cast<unsigned char>(layer->width);
        header.height = static_cast<unsigned char>(layer->height);
        header.minx   = static_cast<unsigned char>(layer->minx);
        header.maxx   = static_cast<unsigned char>(layer->maxx);
        header.miny   = static_cast<unsigned char>(layer->miny);
        header.maxy   = static_cast<unsigned char>(layer->maxy);
        header.hmin   = static_cast<unsigned short>(layer->hmin);
        header.hmax   = static_cast<unsigned short>(layer->hmax);

        TileLayerData* out = &out_tiles[ntiles_out];
        dtStatus st = dtBuildTileCacheLayer(
            &comp, &header,
            layer->heights, layer->areas, layer->cons,
            &out->data, &out->dataSize);
        if (dtStatusSucceed(st)) {
            ++ntiles_out;
        }
    }
    rcFreeHeightfieldLayerSet(lset);
    return ntiles_out;
}

// ─── end TileCache helpers ────────────────────────────────────────────────────

extern "C" {

struct nmBuildSettings {
    float cellSize;
    float cellHeight;
    float agentHeight;
    float agentRadius;
    float agentMaxClimb;
    float agentMaxSlope;
    float regionMinSize;
    float regionMergeSize;
    float edgeMaxLen;
    float edgeMaxError;
    int vertsPerPoly;
    float detailSampleDist;
    float detailSampleMaxError;
    int partitionType;  // 0=Watershed, 1=Monotone, 2=Layers
    int filterLowHangingObstacles;
    int filterLedgeSpans;
    int filterWalkableLowHeightSpans;
};

void* nm_load(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return nullptr;

    NavMeshSetHeader header;
    if (fread(&header, sizeof(header), 1, f) != 1) {
        fclose(f);
        return nullptr;
    }
    if (header.magic != NAVMESHSET_MAGIC || header.version != NAVMESHSET_VERSION) {
        fclose(f);
        return nullptr;
    }

    dtNavMesh* mesh = dtAllocNavMesh();
    if (!mesh) {
        fclose(f);
        return nullptr;
    }
    dtStatus status = mesh->init(&header.params);
    if (dtStatusFailed(status)) {
        dtFreeNavMesh(mesh);
        fclose(f);
        return nullptr;
    }

    for (int i = 0; i < header.numTiles; ++i) {
        NavMeshTileHeader th;
        if (fread(&th, sizeof(th), 1, f) != 1) break;
        if (!th.tileRef || !th.dataSize) break;

        unsigned char* data = static_cast<unsigned char*>(dtAlloc(th.dataSize, DT_ALLOC_PERM));
        if (!data) break;
        memset(data, 0, static_cast<size_t>(th.dataSize));

        if (fread(data, static_cast<size_t>(th.dataSize), 1, f) != 1) {
            dtFree(data);
            break;
        }
        mesh->addTile(data, th.dataSize, DT_TILE_FREE_DATA, th.tileRef, nullptr);
    }

    fclose(f);
    return mesh;
}

int nm_save(void* handle, const char* path) {
    if (!handle || !path) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    FILE* f = fopen(path, "wb");
    if (!f) return -1;

    NavMeshSetHeader header{};
    header.magic = NAVMESHSET_MAGIC;
    header.version = NAVMESHSET_VERSION;
    header.numTiles = 0;

    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = mesh->getTile(i);
        if (tile && tile->header && tile->dataSize > 0) {
            header.numTiles++;
        }
    }

    const dtNavMeshParams* params = mesh->getParams();
    if (!params) {
        fclose(f);
        return -1;
    }
    std::memcpy(&header.params, params, sizeof(dtNavMeshParams));

    if (fwrite(&header, sizeof(header), 1, f) != 1) {
        fclose(f);
        return -1;
    }

    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header || tile->dataSize <= 0) {
            continue;
        }

        NavMeshTileHeader tile_header{};
        tile_header.tileRef = mesh->getTileRef(tile);
        tile_header.dataSize = tile->dataSize;

        if (fwrite(&tile_header, sizeof(tile_header), 1, f) != 1) {
            fclose(f);
            return -1;
        }
        if (fwrite(tile->data, static_cast<size_t>(tile->dataSize), 1, f) != 1) {
            fclose(f);
            return -1;
        }
    }

    fclose(f);
    return 0;
}

void* nm_build_solo_from_obj(
    const char* obj_path,
    const nmBuildSettings* settings,
    int input_is_z_up,
    char* error_out,
    int error_out_len
) {
    fprintf(stderr, "[C++] nm_build_solo_from_obj: Starting\n");
    
    if (!obj_path || !settings) {
        set_error(error_out, error_out_len, "Invalid build arguments.");
        return nullptr;
    }
    if (settings->cellSize <= 0.0f || settings->cellHeight <= 0.0f) {
        set_error(error_out, error_out_len, "Cell size and height must be > 0.");
        return nullptr;
    }
    if (settings->vertsPerPoly < 3 || settings->vertsPerPoly > DT_VERTS_PER_POLYGON) {
        set_error(error_out, error_out_len, "Verts per poly must be in [3, DT_VERTS_PER_POLYGON].");
        return nullptr;
    }

    fprintf(stderr, "[C++] Loading OBJ file: %s\n", obj_path);
    ObjMesh mesh_data;
    if (!load_obj_mesh(obj_path, input_is_z_up != 0, mesh_data, error_out, error_out_len)) {
        fprintf(stderr, "[C++] ERROR: Failed to load OBJ mesh\n");
        return nullptr;
    }

    // Create Recast context - required for all Recast functions
    MinimalRecastContext ctx_wrapper;
    rcContext* ctx = &ctx_wrapper.ctx;

    const float* verts = mesh_data.verts.data();
    const int num_verts = static_cast<int>(mesh_data.verts.size() / 3);
    const int* tris = mesh_data.tris.data();
    const int num_tris = static_cast<int>(mesh_data.tris.size() / 3);
    
    fprintf(stderr, "[C++] Loaded mesh: %d vertices, %d triangles\n", num_verts, num_tris);

    float bounds_min[3]{};
    float bounds_max[3]{};
    calc_bounds(mesh_data.verts, bounds_min, bounds_max);
    
    fprintf(stderr, "[C++] Bounds: min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)\n",
            bounds_min[0], bounds_min[1], bounds_min[2],
            bounds_max[0], bounds_max[1], bounds_max[2]);

    fprintf(stderr, "[C++] Configuring Recast build params\n");
    rcConfig cfg{};
    cfg.cs = settings->cellSize;
    cfg.ch = settings->cellHeight;
    cfg.walkableSlopeAngle = settings->agentMaxSlope;
    cfg.walkableHeight = static_cast<int>(std::ceil(settings->agentHeight / cfg.ch));
    cfg.walkableClimb = static_cast<int>(std::floor(settings->agentMaxClimb / cfg.ch));
    cfg.walkableRadius = static_cast<int>(std::ceil(settings->agentRadius / cfg.cs));
    cfg.maxEdgeLen = static_cast<int>(settings->edgeMaxLen / cfg.cs);
    cfg.maxSimplificationError = settings->edgeMaxError;
    cfg.minRegionArea = static_cast<int>(rcSqr(settings->regionMinSize));
    cfg.mergeRegionArea = static_cast<int>(rcSqr(settings->regionMergeSize));
    cfg.maxVertsPerPoly = settings->vertsPerPoly;
    cfg.detailSampleDist = settings->detailSampleDist < 0.9f ? 0.0f : cfg.cs * settings->detailSampleDist;
    cfg.detailSampleMaxError = cfg.ch * settings->detailSampleMaxError;
    rcVcopy(cfg.bmin, bounds_min);
    rcVcopy(cfg.bmax, bounds_max);
    rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
    
    fprintf(stderr, "[C++] Grid size: %d x %d\n", cfg.width, cfg.height);

    // Declare all variables used in this scope to avoid goto issues
    rcHeightfield* heightfield = nullptr;
    rcCompactHeightfield* compact = nullptr;
    rcContourSet* contours = nullptr;
    rcPolyMesh* poly_mesh = nullptr;
    rcPolyMeshDetail* detail_mesh = nullptr;
    unsigned char* nav_data = nullptr;
    int nav_data_size = 0;
    dtNavMesh* nav_mesh = nullptr;
    std::vector<unsigned char> tri_areas(static_cast<size_t>(num_tris), 0);
    
    // Validate grid size - prevent excessive memory allocation
    if (cfg.width <= 0 || cfg.height <= 0) {
        set_error(error_out, error_out_len, "Invalid grid size (width or height <= 0).");
        goto fail;
    }
    if (cfg.width > 16384 || cfg.height > 16384) {
        set_error(error_out, error_out_len, "Grid size too large (exceeds 16384). Increase cell size.");
        goto fail;
    }
    
    // Estimate memory requirements
    {
        const long long grid_cells = static_cast<long long>(cfg.width) * static_cast<long long>(cfg.height);
        fprintf(stderr, "[C++] Total grid cells: %lld\n", grid_cells);
        if (grid_cells > 100000000) {  // 100 million cells
            set_error(error_out, error_out_len, "Grid too large (>100M cells). Increase cell size.");
            goto fail;
        }
    }

    fprintf(stderr, "[C++] Allocating heightfield\n");
    heightfield = rcAllocHeightfield();
    if (!heightfield) {
        set_error(error_out, error_out_len, "Out of memory: heightfield.");
        goto fail;
    }
    fprintf(stderr, "[C++] Heightfield allocated\n");

    fprintf(stderr, "[C++] Creating heightfield\n");
    if (!rcCreateHeightfield(ctx, *heightfield, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
        set_error(error_out, error_out_len, "Could not create heightfield.");
        goto fail;
    }
    fprintf(stderr, "[C++] Heightfield created successfully\n");

    fprintf(stderr, "[C++] Marking walkable triangles (num_tris=%d)\n", num_tris);
    rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, num_verts, tris, num_tris, tri_areas.data());
    fprintf(stderr, "[C++] Walkable triangles marked\n");
    
    // Validate triangle indices
    fprintf(stderr, "[C++] Validating triangle indices...\n");
    for (int i = 0; i < num_tris; ++i) {
        const int* tri = &tris[i * 3];
        for (int j = 0; j < 3; ++j) {
            if (tri[j] < 0 || tri[j] >= num_verts) {
                char buf[256];
                std::snprintf(buf, sizeof(buf), "Invalid triangle index at tri %d: idx=%d (numVerts=%d)", 
                             i, tri[j], num_verts);
                set_error(error_out, error_out_len, buf);
                goto fail;
            }
        }
    }
    fprintf(stderr, "[C++] Triangle indices validated\n");
    
    // Validate vertex data for NaN/Inf
    fprintf(stderr, "[C++] Validating vertex data...\n");
    for (int i = 0; i < num_verts * 3; ++i) {
        if (!std::isfinite(verts[i])) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Invalid vertex data at index %d: %f", i, verts[i]);
            set_error(error_out, error_out_len, buf);
            goto fail;
        }
    }
    fprintf(stderr, "[C++] Vertex data validated\n");
    
    fprintf(stderr, "[C++] Rasterizing %d triangles into %dx%d grid...\n", num_tris, cfg.width, cfg.height);
    fprintf(stderr, "[C++]   verts ptr: %p, tris ptr: %p, tri_areas ptr: %p\n", 
            (void*)verts, (void*)tris, (void*)tri_areas.data());
    fprintf(stderr, "[C++]   heightfield: spans=%p, width=%d, height=%d\n",
            (void*)heightfield->spans, heightfield->width, heightfield->height);
    fflush(stderr);
    
    if (!rcRasterizeTriangles(ctx, verts, num_verts, tris, tri_areas.data(), num_tris, *heightfield, cfg.walkableClimb)) {
        set_error(error_out, error_out_len, "Could not rasterize triangles.");
        goto fail;
    }
    fprintf(stderr, "[C++] Triangles rasterized successfully\n");

    if (settings->filterLowHangingObstacles) {
        fprintf(stderr, "[C++] Filtering low hanging obstacles\n");
        rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *heightfield);
    }
    if (settings->filterLedgeSpans) {
        fprintf(stderr, "[C++] Filtering ledge spans\n");
        rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield);
    }
    if (settings->filterWalkableLowHeightSpans) {
        fprintf(stderr, "[C++] Filtering walkable low height spans\n");
        rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *heightfield);
    }

    fprintf(stderr, "[C++] Building compact heightfield\n");
    compact = rcAllocCompactHeightfield();
    if (!compact) {
        set_error(error_out, error_out_len, "Out of memory: compact heightfield.");
        goto fail;
    }
    fprintf(stderr, "[C++] Compact heightfield allocated\n");
    
    if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield, *compact)) {
        set_error(error_out, error_out_len, "Could not build compact heightfield.");
        goto fail;
    }
    fprintf(stderr, "[C++] Compact heightfield built\n");
    
    fprintf(stderr, "[C++] Eroding walkable area\n");
    if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *compact)) {
        set_error(error_out, error_out_len, "Could not erode walkable area.");
        goto fail;
    }
    fprintf(stderr, "[C++] Walkable area eroded\n");

    fprintf(stderr, "[C++] Building regions (partition type: %d)\n", settings->partitionType);
    if (settings->partitionType == 0) {
        if (!rcBuildDistanceField(ctx, *compact)) {
            set_error(error_out, error_out_len, "Could not build distance field.");
            goto fail;
        }
        if (!rcBuildRegions(ctx, *compact, 0, cfg.minRegionArea, cfg.mergeRegionArea)) {
            set_error(error_out, error_out_len, "Could not build watershed regions.");
            goto fail;
        }
    } else if (settings->partitionType == 1) {
        if (!rcBuildRegionsMonotone(ctx, *compact, 0, cfg.minRegionArea, cfg.mergeRegionArea)) {
            set_error(error_out, error_out_len, "Could not build monotone regions.");
            goto fail;
        }
    } else {
        if (!rcBuildLayerRegions(ctx, *compact, 0, cfg.minRegionArea)) {
            set_error(error_out, error_out_len, "Could not build layer regions.");
            goto fail;
        }
    }
    fprintf(stderr, "[C++] Regions built\n");

    fprintf(stderr, "[C++] Building contours\n");
    contours = rcAllocContourSet();
    if (!contours) {
        set_error(error_out, error_out_len, "Out of memory: contour set.");
        goto fail;
    }
    if (!rcBuildContours(ctx, *compact, cfg.maxSimplificationError, cfg.maxEdgeLen, *contours)) {
        set_error(error_out, error_out_len, "Could not build contours.");
        goto fail;
    }
    fprintf(stderr, "[C++] Contours built\n");

    fprintf(stderr, "[C++] Building polymesh\n");
    poly_mesh = rcAllocPolyMesh();
    if (!poly_mesh) {
        set_error(error_out, error_out_len, "Out of memory: polymesh.");
        goto fail;
    }
    if (!rcBuildPolyMesh(ctx, *contours, cfg.maxVertsPerPoly, *poly_mesh)) {
        set_error(error_out, error_out_len, "Could not build polymesh.");
        goto fail;
    }
    fprintf(stderr, "[C++] Polymesh built\n");

    fprintf(stderr, "[C++] Building detail mesh\n");
    detail_mesh = rcAllocPolyMeshDetail();
    if (!detail_mesh) {
        set_error(error_out, error_out_len, "Out of memory: detail mesh.");
        goto fail;
    }
    if (!rcBuildPolyMeshDetail(ctx, *poly_mesh, *compact, cfg.detailSampleDist, cfg.detailSampleMaxError, *detail_mesh)) {
        set_error(error_out, error_out_len, "Could not build detail mesh.");
        goto fail;
    }
    fprintf(stderr, "[C++] Detail mesh built\n");

    fprintf(stderr, "[C++] Setting polygon flags\n");
    for (int i = 0; i < poly_mesh->npolys; ++i) {
        if (poly_mesh->areas[i] == RC_WALKABLE_AREA) {
            poly_mesh->areas[i] = 0;
            poly_mesh->flags[i] = 1;
        } else {
            poly_mesh->flags[i] = 0;
        }
    }

    fprintf(stderr, "[C++] Creating Detour navmesh data\n");
    {
        dtNavMeshCreateParams params{};
        params.verts = poly_mesh->verts;
        params.vertCount = poly_mesh->nverts;
        params.polys = poly_mesh->polys;
        params.polyAreas = poly_mesh->areas;
        params.polyFlags = poly_mesh->flags;
        params.polyCount = poly_mesh->npolys;
        params.nvp = poly_mesh->nvp;
        params.detailMeshes = detail_mesh->meshes;
        params.detailVerts = detail_mesh->verts;
        params.detailVertsCount = detail_mesh->nverts;
        params.detailTris = detail_mesh->tris;
        params.detailTriCount = detail_mesh->ntris;
        params.walkableHeight = settings->agentHeight;
        params.walkableRadius = settings->agentRadius;
        params.walkableClimb = settings->agentMaxClimb;
        rcVcopy(params.bmin, poly_mesh->bmin);
        rcVcopy(params.bmax, poly_mesh->bmax);
        params.cs = cfg.cs;
        params.ch = cfg.ch;
        params.buildBvTree = true;

        if (!dtCreateNavMeshData(&params, &nav_data, &nav_data_size)) {
            set_error(error_out, error_out_len, "Could not build Detour navmesh data.");
            goto fail;
        }
        fprintf(stderr, "[C++] Detour navmesh data created (size: %d bytes)\n", nav_data_size);
    }

    fprintf(stderr, "[C++] Allocating and initializing navmesh\n");
    nav_mesh = dtAllocNavMesh();
    if (!nav_mesh) {
        set_error(error_out, error_out_len, "Could not allocate Detour navmesh.");
        goto fail;
    }

    if (dtStatusFailed(nav_mesh->init(nav_data, nav_data_size, DT_TILE_FREE_DATA))) {
        set_error(error_out, error_out_len, "Could not initialize Detour navmesh.");
        goto fail;
    }

    nav_data = nullptr;  // owned by nav_mesh

    fprintf(stderr, "[C++] Cleaning up temporary data structures\n");
    if (detail_mesh) rcFreePolyMeshDetail(detail_mesh);
    if (poly_mesh) rcFreePolyMesh(poly_mesh);
    if (contours) rcFreeContourSet(contours);
    if (compact) rcFreeCompactHeightfield(compact);
    if (heightfield) rcFreeHeightField(heightfield);
    set_error(error_out, error_out_len, "");
    fprintf(stderr, "[C++] SUCCESS - navmesh built successfully\n");
    return nav_mesh;

fail:
    fprintf(stderr, "[C++] FAILURE - cleaning up after error\n");
    if (nav_data) dtFree(nav_data);
    if (nav_mesh) dtFreeNavMesh(nav_mesh);
    if (detail_mesh) rcFreePolyMeshDetail(detail_mesh);
    if (poly_mesh) rcFreePolyMesh(poly_mesh);
    if (contours) rcFreeContourSet(contours);
    if (compact) rcFreeCompactHeightfield(compact);
    if (heightfield) rcFreeHeightField(heightfield);
    return nullptr;
}

void nm_free(void* handle) {
    if (handle) dtFreeNavMesh(static_cast<dtNavMesh*>(handle));
}

int nm_tile_count(void* handle) {
    if (!handle) return 0;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);
    int count = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) ++count;
    }
    return count;
}

int nm_tile_detail_verts(void* handle, int tile_idx, float* out, int max_floats) {
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) {
                tile = t;
                break;
            }
        }
    }
    if (!tile) return -1;

    const dtMeshHeader* h = tile->header;
    int total_verts = h->vertCount + h->detailVertCount;
    int needed = total_verts * 3;
    if (max_floats == 0) return needed;
    if (max_floats < needed) return -1;

    for (int v = 0; v < h->vertCount; ++v) {
        out[v * 3 + 0] = tile->verts[v * 3 + 0];
        out[v * 3 + 1] = tile->verts[v * 3 + 1];
        out[v * 3 + 2] = tile->verts[v * 3 + 2];
    }
    for (int v = 0; v < h->detailVertCount; ++v) {
        int base = h->vertCount + v;
        out[base * 3 + 0] = tile->detailVerts[v * 3 + 0];
        out[base * 3 + 1] = tile->detailVerts[v * 3 + 1];
        out[base * 3 + 2] = tile->detailVerts[v * 3 + 2];
    }
    return needed;
}

int nm_tile_detail_tris(void* handle, int tile_idx, int* out, int max_ints) {
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) {
                tile = t;
                break;
            }
        }
    }
    if (!tile) return -1;

    const dtMeshHeader* h = tile->header;
    int needed = h->detailTriCount * 3;
    if (max_ints == 0) return needed;
    if (max_ints < needed) return -1;

    int out_idx = 0;
    for (int m = 0; m < h->detailMeshCount; ++m) {
        const dtPolyDetail& dm = tile->detailMeshes[m];
        const dtPoly& poly = tile->polys[m];

        for (unsigned int t = 0; t < dm.triCount; ++t) {
            const unsigned char* tri = &tile->detailTris[(dm.triBase + t) * 4];

            for (int k = 0; k < 3; ++k) {
                unsigned char vi = tri[k];
                int abs_idx;
                if (vi < poly.vertCount) {
                    abs_idx = poly.verts[vi];
                } else {
                    abs_idx = h->vertCount + static_cast<int>(dm.vertBase + (vi - poly.vertCount));
                }
                out[out_idx++] = abs_idx;
            }
        }
    }
    return out_idx;
}

int nm_tile_bounds(void* handle, int tile_idx, float* out_min, float* out_max) {
    if (!handle) return -1;
    const dtNavMesh* mesh = static_cast<const dtNavMesh*>(handle);

    int valid = -1;
    const dtMeshTile* tile = nullptr;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = mesh->getTile(i);
        if (t && t->header && t->dataSize > 0) {
            if (++valid == tile_idx) {
                tile = t;
                break;
            }
        }
    }
    if (!tile) return -1;

    for (int k = 0; k < 3; ++k) {
        out_min[k] = tile->header->bmin[k];
        out_max[k] = tile->header->bmax[k];
    }
    return 0;
}

void* nm_query_create(void* handle, int max_nodes) {
    if (!handle) return nullptr;
    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    if (!q) return nullptr;
    dtStatus s = q->init(static_cast<dtNavMesh*>(handle), max_nodes);
    if (dtStatusFailed(s)) {
        dtFreeNavMeshQuery(q);
        return nullptr;
    }
    return q;
}

void nm_query_free(void* qhandle) {
    if (qhandle) dtFreeNavMeshQuery(static_cast<dtNavMeshQuery*>(qhandle));
}

uint32_t nm_find_nearest_poly(void* qhandle, const float* pos, const float* extents, float* nearest_pt_out) {
    if (!qhandle) return 0;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);

    dtPolyRef ref = 0;
    float pt[3] = {0, 0, 0};
    dtStatus s = q->findNearestPoly(pos, extents, &filter, &ref, pt);
    if (dtStatusFailed(s)) return 0;

    if (nearest_pt_out) {
        nearest_pt_out[0] = pt[0];
        nearest_pt_out[1] = pt[1];
        nearest_pt_out[2] = pt[2];
    }
    return static_cast<uint32_t>(ref);
}

int nm_find_path(
    void* qhandle,
    uint32_t start_ref,
    uint32_t end_ref,
    const float* start_pos,
    const float* end_pos,
    uint32_t* poly_path_out,
    int max_path
) {
    if (!qhandle || !poly_path_out) return -1;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    dtQueryFilter filter;
    filter.setIncludeFlags(0xFFFF);
    filter.setExcludeFlags(0);

    int npath = 0;
    static_assert(sizeof(dtPolyRef) == sizeof(uint32_t), "polyref size mismatch");

    dtStatus s = q->findPath(
        static_cast<dtPolyRef>(start_ref),
        static_cast<dtPolyRef>(end_ref),
        start_pos,
        end_pos,
        &filter,
        reinterpret_cast<dtPolyRef*>(poly_path_out),
        &npath,
        max_path
    );

    if (dtStatusFailed(s)) return -1;
    return npath;
}

int nm_find_straight_path(
    void* qhandle,
    const float* start_pos,
    const float* end_pos,
    const uint32_t* poly_corridor,
    int corridor_len,
    float* straight_path_out,
    int max_pts
) {
    if (!qhandle || !straight_path_out) return -1;
    dtNavMeshQuery* q = static_cast<dtNavMeshQuery*>(qhandle);

    int nstraight = 0;
    dtStatus s = q->findStraightPath(
        start_pos,
        end_pos,
        reinterpret_cast<const dtPolyRef*>(poly_corridor),
        corridor_len,
        straight_path_out,
        nullptr,
        nullptr,
        &nstraight,
        max_pts
    );

    if (dtStatusFailed(s)) return -1;
    return nstraight;
}

void* nm_crowd_create(void* navhandle, int max_agents, float max_agent_radius) {
    if (!navhandle) return nullptr;
    dtCrowd* crowd = dtAllocCrowd();
    if (!crowd) return nullptr;
    if (!crowd->init(max_agents, max_agent_radius, static_cast<dtNavMesh*>(navhandle))) {
        dtFreeCrowd(crowd);
        return nullptr;
    }
    return crowd;
}

void nm_crowd_destroy(void* crowdhandle) {
    if (crowdhandle) dtFreeCrowd(static_cast<dtCrowd*>(crowdhandle));
}

int nm_crowd_add_agent(void* crowdhandle, const float* pos, const dtCrowdAgentParams* params) {
    if (!crowdhandle) return -1;
    return static_cast<dtCrowd*>(crowdhandle)->addAgent(pos, params);
}

void nm_crowd_remove_agent(void* crowdhandle, int idx) {
    if (!crowdhandle) return;
    static_cast<dtCrowd*>(crowdhandle)->removeAgent(idx);
}

bool nm_crowd_request_move_target(void* crowdhandle, int idx, uint32_t poly_ref, const float* pos) {
    if (!crowdhandle) return false;
    return static_cast<dtCrowd*>(crowdhandle)->requestMoveTarget(idx, static_cast<dtPolyRef>(poly_ref), pos);
}

void nm_crowd_update(void* crowdhandle, float dt) {
    if (!crowdhandle) return;
    static_cast<dtCrowd*>(crowdhandle)->update(dt, nullptr);
}

bool nm_crowd_get_agent_pos(void* crowdhandle, int idx, float* pos_out, float* vel_out) {
    if (!crowdhandle) return false;
    const dtCrowdAgent* agent = static_cast<dtCrowd*>(crowdhandle)->getAgent(idx);
    if (!agent || !agent->active) return false;
    if (pos_out) {
        pos_out[0] = agent->npos[0];
        pos_out[1] = agent->npos[1];
        pos_out[2] = agent->npos[2];
    }
    if (vel_out) {
        vel_out[0] = agent->vel[0];
        vel_out[1] = agent->vel[1];
        vel_out[2] = agent->vel[2];
    }
    return true;
}

bool nm_crowd_request_move_velocity(void* crowdhandle, int idx, const float* vel) {
    if (!crowdhandle) return false;
    return static_cast<dtCrowd*>(crowdhandle)->requestMoveVelocity(idx, vel);
}

bool nm_crowd_teleport_agent(void* crowdhandle, int idx, const float* pos) {
    if (!crowdhandle || !pos) return false;
    dtCrowd* crowd = static_cast<dtCrowd*>(crowdhandle);
    dtCrowdAgent* agent = crowd->getEditableAgent(idx);
    if (!agent || !agent->active) return false;

    dtPolyRef ref = 0;
    float nearest[3] = {pos[0], pos[1], pos[2]};
    const dtQueryFilter* filter = crowd->getFilter(agent->params.queryFilterType);
    const float* extents = crowd->getQueryHalfExtents();
    const dtNavMeshQuery* navquery_const = crowd->getNavMeshQuery();
    dtNavMeshQuery* navquery = const_cast<dtNavMeshQuery*>(navquery_const);
    if (navquery && filter && extents) {
        const dtStatus s = navquery->findNearestPoly(pos, extents, filter, &ref, nearest);
        if (dtStatusFailed(s)) {
            ref = 0;
            nearest[0] = pos[0];
            nearest[1] = pos[1];
            nearest[2] = pos[2];
        }
    }

    crowd->resetMoveTarget(idx);
    agent->corridor.reset(ref, nearest);
    agent->boundary.reset();
    agent->partial = false;
    agent->topologyOptTime = 0.0f;
    agent->targetReplanTime = 0.0f;
    agent->nneis = 0;
    agent->desiredSpeed = 0.0f;

    agent->npos[0] = nearest[0];
    agent->npos[1] = nearest[1];
    agent->npos[2] = nearest[2];
    agent->dvel[0] = agent->dvel[1] = agent->dvel[2] = 0.0f;
    agent->nvel[0] = agent->nvel[1] = agent->nvel[2] = 0.0f;
    agent->vel[0] = agent->vel[1] = agent->vel[2] = 0.0f;
    agent->state = ref ? DT_CROWDAGENT_STATE_WALKING : DT_CROWDAGENT_STATE_INVALID;
    return ref != 0;
}

void nm_crowd_force_agent_pos(void* crowdhandle, int idx, const float* pos) {
    if (!crowdhandle) return;
    dtCrowdAgent* agent = static_cast<dtCrowd*>(crowdhandle)->getEditableAgent(idx);
    if (!agent || !agent->active) return;
    agent->npos[0] = pos[0];
    agent->npos[1] = pos[1];
    agent->npos[2] = pos[2];
}

// Snap position to nearest navmesh poly and update the path corridor in-place.
// Unlike force_agent_pos, this keeps the existing move target valid.
// half_extents: optional custom search box (Y-up). Pass NULL to use crowd defaults.
// For multi-floor environments, pass a tight vertical extent to avoid cross-floor snaps.
bool nm_crowd_sync_agent_pos(void* crowdhandle, int idx, const float* pos, const float* half_extents) {
    if (!crowdhandle || !pos) return false;
    dtCrowd* crowd = static_cast<dtCrowd*>(crowdhandle);
    dtCrowdAgent* agent = crowd->getEditableAgent(idx);
    if (!agent || !agent->active) return false;

    float nearest[3] = {pos[0], pos[1], pos[2]};
    dtPolyRef ref = 0;
    const dtQueryFilter* filter = crowd->getFilter(agent->params.queryFilterType);
    const float* extents = half_extents ? half_extents : crowd->getQueryHalfExtents();
    const dtNavMeshQuery* q_const = crowd->getNavMeshQuery();
    dtNavMeshQuery* navquery = const_cast<dtNavMeshQuery*>(q_const);

    if (navquery && filter && extents) {
        const dtStatus s = navquery->findNearestPoly(pos, extents, filter, &ref, nearest);
        if (dtStatusFailed(s)) ref = 0;
    }
    if (!ref) return false;

    agent->npos[0] = nearest[0];
    agent->npos[1] = nearest[1];
    agent->npos[2] = nearest[2];
    agent->corridor.movePosition(nearest, navquery, filter);
    agent->state = DT_CROWDAGENT_STATE_WALKING;
    return true;
}

void nm_crowd_set_obstacle_avoidance_params(void* crowdhandle, int idx, const dtObstacleAvoidanceParams* params) {
    if (!crowdhandle || !params) return;
    dtCrowd* crowd = static_cast<dtCrowd*>(crowdhandle);
    crowd->setObstacleAvoidanceParams(idx, params);
}

bool nm_crowd_get_obstacle_avoidance_params(void* crowdhandle, int idx, dtObstacleAvoidanceParams* out_params) {
    if (!crowdhandle || !out_params) return false;
    dtCrowd* crowd = static_cast<dtCrowd*>(crowdhandle);
    const dtObstacleAvoidanceParams* params = crowd->getObstacleAvoidanceParams(idx);
    if (!params) return false;
    *out_params = *params;
    return true;
}

// ─── TileCache API ────────────────────────────────────────────────────────────

// Build a tiled navmesh + tile cache from an OBJ file.
// tile_size: voxels per tile edge (32-64 typical).
// max_obstacles: maximum simultaneous obstacles in the tile cache pool.
// Returns an opaque TileCacheHandle* or nullptr on failure.
void* nm_build_tiled_with_cache(
    const char*            obj_path,
    const nmBuildSettings* settings,
    int                    input_is_z_up,
    int                    tile_size,
    int                    max_obstacles,
    char*                  error_out,
    int                    error_out_len
) {
    if (!obj_path || !settings || tile_size < 1 || max_obstacles < 1) {
        set_error(error_out, error_out_len, "Invalid arguments to nm_build_tiled_with_cache.");
        return nullptr;
    }

    ObjMesh mesh_data;
    if (!load_obj_mesh(obj_path, input_is_z_up != 0, mesh_data, error_out, error_out_len))
        return nullptr;

    const float* verts = mesh_data.verts.data();
    const int    nverts = static_cast<int>(mesh_data.verts.size() / 3);
    const int*   tris  = mesh_data.tris.data();
    const int    ntris = static_cast<int>(mesh_data.tris.size() / 3);

    float bmin[3], bmax[3];
    calc_bounds(mesh_data.verts, bmin, bmax);

    float cs  = settings->cellSize;
    float ch  = settings->cellHeight;

    // Per-tile grid dimensions
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, cs, &gw, &gh);
    const int tw = (gw + tile_size - 1) / tile_size;
    const int th = (gh + tile_size - 1) / tile_size;

    // Tile ref bits
    const int EXPECTED_LAYERS = 4;
    int tileBits = dtIlog2(dtNextPow2(tw * th * EXPECTED_LAYERS));
    if (tileBits > 14) tileBits = 14;
    const int polyBits        = 22 - tileBits;
    const int maxTiles        = 1 << tileBits;
    const int maxPolysPerTile = 1 << polyBits;

    // Build rcConfig for per-tile rasterization
    rcConfig cfg{};
    cfg.cs                   = cs;
    cfg.ch                   = ch;
    cfg.walkableSlopeAngle   = settings->agentMaxSlope;
    cfg.walkableHeight       = static_cast<int>(std::ceil(settings->agentHeight / ch));
    cfg.walkableClimb        = static_cast<int>(std::floor(settings->agentMaxClimb / ch));
    cfg.walkableRadius       = static_cast<int>(std::ceil(settings->agentRadius / cs));
    cfg.maxEdgeLen           = static_cast<int>(settings->edgeMaxLen / cs);
    cfg.maxSimplificationError = settings->edgeMaxError;
    cfg.minRegionArea        = static_cast<int>(rcSqr(settings->regionMinSize));
    cfg.mergeRegionArea      = static_cast<int>(rcSqr(settings->regionMergeSize));
    cfg.maxVertsPerPoly      = settings->vertsPerPoly;
    cfg.detailSampleDist     = settings->detailSampleDist < 0.9f ? 0.0f : cs * settings->detailSampleDist;
    cfg.detailSampleMaxError = ch * settings->detailSampleMaxError;
    cfg.tileSize             = tile_size;
    cfg.borderSize           = cfg.walkableRadius + 3;  // padding to prevent edge artefacts
    cfg.width                = cfg.tileSize + cfg.borderSize * 2;
    cfg.height               = cfg.tileSize + cfg.borderSize * 2;
    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);

    // Allocate subobjects owned by the handle
    auto* alloc    = new TCLinearAllocator(32 * 1024);
    auto* comp     = new TCFastLZCompressor();
    auto* meshproc = new TCSimpleMeshProcess();

    // Init tile cache
    dtTileCacheParams tcparams{};
    rcVcopy(tcparams.orig, bmin);
    tcparams.cs                   = cs;
    tcparams.ch                   = ch;
    tcparams.width                = tile_size;
    tcparams.height               = tile_size;
    tcparams.walkableHeight       = settings->agentHeight;
    tcparams.walkableRadius       = settings->agentRadius;
    tcparams.walkableClimb        = settings->agentMaxClimb;
    tcparams.maxSimplificationError = settings->edgeMaxError;
    tcparams.maxTiles             = tw * th * EXPECTED_LAYERS;
    tcparams.maxObstacles         = static_cast<int>(max_obstacles);

    dtTileCache* tc = dtAllocTileCache();
    if (!tc) {
        set_error(error_out, error_out_len, "Could not allocate dtTileCache.");
        delete alloc; delete comp; delete meshproc;
        return nullptr;
    }
    if (dtStatusFailed(tc->init(&tcparams, alloc, comp, meshproc))) {
        set_error(error_out, error_out_len, "dtTileCache::init failed.");
        dtFreeTileCache(tc); delete alloc; delete comp; delete meshproc;
        return nullptr;
    }

    // Init navmesh
    dtNavMeshParams nmparams{};
    rcVcopy(nmparams.orig, bmin);
    nmparams.tileWidth  = static_cast<float>(tile_size) * cs;
    nmparams.tileHeight = static_cast<float>(tile_size) * cs;
    nmparams.maxTiles   = maxTiles;
    nmparams.maxPolys   = maxPolysPerTile;

    dtNavMesh* nav = dtAllocNavMesh();
    if (!nav) {
        set_error(error_out, error_out_len, "Could not allocate dtNavMesh.");
        dtFreeTileCache(tc); delete alloc; delete comp; delete meshproc;
        return nullptr;
    }
    if (dtStatusFailed(nav->init(&nmparams))) {
        set_error(error_out, error_out_len, "dtNavMesh::init failed.");
        dtFreeNavMesh(nav); dtFreeTileCache(tc); delete alloc; delete comp; delete meshproc;
        return nullptr;
    }

    // Rasterize all tiles and populate tile cache
    MinimalRecastContext ctx_wrap;
    rcContext* ctx = &ctx_wrap.ctx;

    bool filter_low   = settings->filterLowHangingObstacles != 0;
    bool filter_ledge = settings->filterLedgeSpans != 0;
    bool filter_lowhgt = settings->filterWalkableLowHeightSpans != 0;

    for (int ty = 0; ty < th; ++ty) {
        for (int tx = 0; tx < tw; ++tx) {
            TileLayerData layers[TC_MAX_LAYERS]{};
            int nlayers = rasterize_tile_layers(
                ctx, verts, nverts, tris, ntris,
                cfg, filter_low, filter_ledge, filter_lowhgt,
                tx, ty, layers, TC_MAX_LAYERS);

            for (int i = 0; i < nlayers; ++i) {
                dtStatus st = tc->addTile(
                    layers[i].data, layers[i].dataSize,
                    DT_COMPRESSEDTILE_FREE_DATA, nullptr);
                if (dtStatusFailed(st)) {
                    dtFree(layers[i].data);
                    layers[i].data = nullptr;
                }
            }
        }
    }

    // Build initial navmesh tiles from cache
    for (int ty = 0; ty < th; ++ty)
        for (int tx = 0; tx < tw; ++tx)
            tc->buildNavMeshTilesAt(tx, ty, nav);

    auto* handle    = new TileCacheHandle();
    handle->tc      = tc;
    handle->nav     = nav;
    handle->alloc   = alloc;
    handle->comp    = comp;
    handle->meshproc = meshproc;

    set_error(error_out, error_out_len, "");
    return handle;
}

int nm_tc_save(void* tc_handle, const char* path) {
    if (!tc_handle || !path) return -1;
    auto* h = static_cast<TileCacheHandle*>(tc_handle);

    FILE* fp = fopen(path, "wb");
    if (!fp) return -1;

    TileCacheSetHeader header{};
    header.magic   = TILECACHESET_MAGIC;
    header.version = TILECACHESET_VERSION;
    header.numTiles = 0;
    for (int i = 0; i < h->tc->getTileCount(); ++i) {
        const dtCompressedTile* t = h->tc->getTile(i);
        if (t && t->header && t->dataSize > 0) ++header.numTiles;
    }
    std::memcpy(&header.cacheParams, h->tc->getParams(), sizeof(dtTileCacheParams));
    std::memcpy(&header.meshParams,  h->nav->getParams(), sizeof(dtNavMeshParams));

    if (fwrite(&header, sizeof(header), 1, fp) != 1) { fclose(fp); return -1; }

    for (int i = 0; i < h->tc->getTileCount(); ++i) {
        const dtCompressedTile* t = h->tc->getTile(i);
        if (!t || !t->header || t->dataSize <= 0) continue;
        TileCacheTileHeader th{};
        th.tileRef  = h->tc->getTileRef(t);
        th.dataSize = t->dataSize;
        if (fwrite(&th, sizeof(th), 1, fp) != 1) { fclose(fp); return -1; }
        if (fwrite(t->data, static_cast<size_t>(t->dataSize), 1, fp) != 1) { fclose(fp); return -1; }
    }
    fclose(fp);
    return 0;
}

void* nm_tc_load(const char* path) {
    if (!path) return nullptr;
    FILE* fp = fopen(path, "rb");
    if (!fp) return nullptr;

    TileCacheSetHeader header{};
    if (fread(&header, sizeof(header), 1, fp) != 1) { fclose(fp); return nullptr; }
    if (header.magic != TILECACHESET_MAGIC || header.version != TILECACHESET_VERSION) {
        fclose(fp); return nullptr;
    }

    auto* alloc    = new TCLinearAllocator(32 * 1024);
    auto* comp     = new TCFastLZCompressor();
    auto* meshproc = new TCSimpleMeshProcess();

    dtNavMesh* nav = dtAllocNavMesh();
    if (!nav || dtStatusFailed(nav->init(&header.meshParams))) {
        if (nav) dtFreeNavMesh(nav);
        delete alloc; delete comp; delete meshproc;
        fclose(fp); return nullptr;
    }

    dtTileCache* tc = dtAllocTileCache();
    if (!tc || dtStatusFailed(tc->init(&header.cacheParams, alloc, comp, meshproc))) {
        if (tc) dtFreeTileCache(tc);
        dtFreeNavMesh(nav); delete alloc; delete comp; delete meshproc;
        fclose(fp); return nullptr;
    }

    for (int i = 0; i < header.numTiles; ++i) {
        TileCacheTileHeader th{};
        if (fread(&th, sizeof(th), 1, fp) != 1) break;
        if (!th.tileRef || !th.dataSize) break;

        unsigned char* data = static_cast<unsigned char*>(dtAlloc(th.dataSize, DT_ALLOC_PERM));
        if (!data) break;
        std::memset(data, 0, static_cast<size_t>(th.dataSize));
        if (fread(data, static_cast<size_t>(th.dataSize), 1, fp) != 1) {
            dtFree(data); break;
        }

        dtCompressedTileRef tile_ref = 0;
        dtStatus st = tc->addTile(data, th.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile_ref);
        if (dtStatusFailed(st)) {
            dtFree(data);
            continue;
        }
        if (tile_ref) tc->buildNavMeshTile(tile_ref, nav);
    }

    fclose(fp);

    auto* handle     = new TileCacheHandle();
    handle->tc       = tc;
    handle->nav      = nav;
    handle->alloc    = alloc;
    handle->comp     = comp;
    handle->meshproc = meshproc;
    return handle;
}

void nm_tc_free(void* tc_handle) {
    if (!tc_handle) return;
    auto* h = static_cast<TileCacheHandle*>(tc_handle);
    if (h->tc)  dtFreeTileCache(h->tc);
    if (h->nav) dtFreeNavMesh(h->nav);
    delete h->alloc;
    delete h->comp;
    delete h->meshproc;
    delete h;
}

// Returns the dtNavMesh* inside the handle. Lifetime owned by the handle.
void* nm_tc_get_navmesh(void* tc_handle) {
    if (!tc_handle) return nullptr;
    return static_cast<TileCacheHandle*>(tc_handle)->nav;
}

// Add a cylinder obstacle. pos[3] is Y-up; pos[1] is the BASE (bottom face) of the cylinder.
// Returns obstacle ref (non-zero) on success, 0 on failure.
uint32_t nm_tc_add_cylinder(void* tc_handle, const float* pos, float radius, float height) {
    if (!tc_handle || !pos) return 0;
    auto* h = static_cast<TileCacheHandle*>(tc_handle);
    dtObstacleRef ref = 0;
    dtStatus st = h->tc->addObstacle(pos, radius, height, &ref);
    if (dtStatusFailed(st)) return 0;
    return static_cast<uint32_t>(ref);
}

void nm_tc_remove_obstacle(void* tc_handle, uint32_t ref) {
    if (!tc_handle || ref == 0) return;
    static_cast<TileCacheHandle*>(tc_handle)->tc->removeObstacle(
        static_cast<dtObstacleRef>(ref));
}

// Rebuild tiles touched by pending obstacle requests.
// upToDate_out is set to 1 when no more pending work remains.
void nm_tc_update(void* tc_handle, float dt, int* upToDate_out) {
    if (!tc_handle) return;
    auto* h = static_cast<TileCacheHandle*>(tc_handle);
    bool upToDate = false;
    h->tc->update(dt, h->nav, &upToDate);
    if (upToDate_out) *upToDate_out = upToDate ? 1 : 0;
}

int nm_tc_tile_count(void* tc_handle) {
    if (!tc_handle) return 0;
    return nm_tile_count(static_cast<TileCacheHandle*>(tc_handle)->nav);
}

}  // extern "C"
