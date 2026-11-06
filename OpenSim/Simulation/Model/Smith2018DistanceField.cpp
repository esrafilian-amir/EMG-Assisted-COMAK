/* -------------------------------------------------------------------------- *
 *                    OpenSim:  Smith2018DistanceField.cpp                    *
 * -------------------------------------------------------------------------- */

#include "Smith2018DistanceField.h"

#include <OpenSim/Common/Exception.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>

using SimTK::Vec3;

namespace OpenSim {

namespace {

enum Region { FACE = 0, VERT_A, VERT_B, VERT_C, EDGE_AB, EDGE_BC, EDGE_CA };

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection)
// and the feature (face, vertex or edge) it lies on.
Vec3 closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b,
    const Vec3& c, Region& region)
{
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = SimTK::dot(ab, ap), d2 = SimTK::dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { region = VERT_A; return a; }

    const Vec3 bp = p - b;
    const double d3 = SimTK::dot(ab, bp), d4 = SimTK::dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { region = VERT_B; return b; }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        region = EDGE_AB;
        return a + ab * (d1 / (d1 - d3));
    }

    const Vec3 cp = p - c;
    const double d5 = SimTK::dot(ab, cp), d6 = SimTK::dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { region = VERT_C; return c; }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        region = EDGE_CA;
        return a + ac * (d2 / (d2 - d6));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        region = EDGE_BC;
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }

    const double denom = 1.0 / (va + vb + vc);
    region = FACE;
    return a + ab * (vb * denom) + ac * (vc * denom);
}

inline std::uint64_t edgeKey(int a, int b)
{
    if (a > b) { std::swap(a, b); }
    return (std::uint64_t(a) << 32) | std::uint32_t(b);
}

inline double boxDist2(const Vec3& p, const Vec3& lo, const Vec3& hi)
{
    double d2 = 0;
    for (int d = 0; d < 3; ++d) {
        const double e = std::max(std::max(lo[d] - p[d], p[d] - hi[d]), 0.0);
        d2 += e * e;
    }
    return d2;
}

} // namespace

Smith2018DistanceField::Smith2018DistanceField(
    const SimTK::Vector_<Vec3>& vertices,
    const std::vector<std::vector<int>>& faces,
    double voxel_size, double negative_band, double edge_taper_width)
    : _h(voxel_size), _band(negative_band), _taper(edge_taper_width)
{
    OPENSIM_THROW_IF(!(voxel_size > 0.0), Exception,
        "Smith2018DistanceField: the voxel size must be positive.");
    OPENSIM_THROW_IF(faces.empty() || vertices.size() == 0, Exception,
        "Smith2018DistanceField: the mesh is empty.");

    const int nv = (int)vertices.size();
    const int nf = (int)faces.size();
    _v.resize(nv);
    for (int i = 0; i < nv; ++i) { _v[i] = vertices[i]; }
    _f.resize(nf);
    for (int f = 0; f < nf; ++f) {
        _f[f] = {faces[f][0], faces[f][1], faces[f][2]};
    }

    // Face normals, edge and vertex pseudo-normals ------------------------
    _face_normal.assign(nf, Vec3(0));
    _vert_normal.assign(nv, Vec3(0));
    _vert_rim.assign(nv, 0);
    _edges.reserve(size_t(nf) * 2);

    for (int f = 0; f < nf; ++f) {
        const int* v = _f[f].data();
        Vec3 n = (_v[v[1]] - _v[v[0]]) % (_v[v[2]] - _v[v[0]]);
        const double len = n.norm();
        n = (len > 0) ? Vec3(n / len) : Vec3(0);
        _face_normal[f] = n;

        for (int k = 0; k < 3; ++k) {
            const int i0 = v[k], i1 = v[(k + 1) % 3], i2 = v[(k + 2) % 3];
            const Vec3 e1 = _v[i1] - _v[i0];
            const Vec3 e2 = _v[i2] - _v[i0];
            const double l1 = e1.norm(), l2 = e2.norm();
            double angle = 0;
            if (l1 > 0 && l2 > 0) {
                angle = std::acos(std::max(-1.0,
                    std::min(1.0, SimTK::dot(e1, e2) / (l1 * l2))));
            }
            _vert_normal[i0] += n * angle;

            EdgeInfo& e = _edges[edgeKey(i0, i1)];
            e.normal_sum += n;
            e.count++;
        }
    }
    for (const auto& kv : _edges) {
        if (kv.second.count == 1) {
            _num_boundary_edges++;
            _vert_rim[int(kv.first >> 32)] = 1;
            _vert_rim[int(kv.first & 0xffffffffu)] = 1;
            _boundary_edges.push_back({_v[int(kv.first >> 32)],
                                       _v[int(kv.first & 0xffffffffu)]});
        }
    }

    // Bounding volume hierarchy ---------------------------------------------
    _bvh_tri.resize(nf);
    for (int f = 0; f < nf; ++f) { _bvh_tri[f] = f; }
    _bvh.reserve(size_t(2) * nf);
    buildBvh(0, nf);

    // Grid (the nodes are computed when they are needed) -----------------------
    Vec3 lo(std::numeric_limits<double>::max());
    Vec3 hi(-std::numeric_limits<double>::max());
    for (int i = 0; i < nv; ++i) {
        for (int d = 0; d < 3; ++d) {
            lo[d] = std::min(lo[d], _v[i][d]);
            hi[d] = std::max(hi[d], _v[i][d]);
        }
    }
    const double margin = negative_band + 2.0 * voxel_size;
    _origin = lo - Vec3(margin);
    long long nblocks = 1;
    for (int d = 0; d < 3; ++d) {
        _n[d] = (int)std::ceil((hi[d] - lo[d] + 2 * margin) / voxel_size) + 2;
        _nb[d] = (_n[d] + B - 1) / B;
        nblocks *= _nb[d];
    }
    OPENSIM_THROW_IF(nblocks > 200000000LL, Exception,
        "Smith2018DistanceField: the grid would have " +
        std::to_string(nblocks) + " blocks. Increase the voxel size or reduce "
        "max_proximity.");
    _blocks.resize((size_t)nblocks);
    _block_ready = std::vector<std::atomic<unsigned char>>((size_t)nblocks);
    for (auto& r : _block_ready) { r.store(0); }
}

int Smith2018DistanceField::buildBvh(int start, int count)
{
    const int id = (int)_bvh.size();
    _bvh.emplace_back();
    Vec3 lo(std::numeric_limits<double>::max());
    Vec3 hi(-std::numeric_limits<double>::max());
    Vec3 clo = lo, chi = hi;
    for (int t = start; t < start + count; ++t) {
        const std::array<int, 3>& tri = _f[_bvh_tri[t]];
        const Vec3 c = (_v[tri[0]] + _v[tri[1]] + _v[tri[2]]) / 3.0;
        for (int k = 0; k < 3; ++k) {
            const Vec3& p = _v[tri[k]];
            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
        for (int d = 0; d < 3; ++d) {
            clo[d] = std::min(clo[d], c[d]);
            chi[d] = std::max(chi[d], c[d]);
        }
    }
    _bvh[id].lo = lo;
    _bvh[id].hi = hi;
    if (count <= 4) {
        _bvh[id].start = start;
        _bvh[id].count = count;
        return id;
    }
    int axis = 0;
    for (int d = 1; d < 3; ++d) {
        if (chi[d] - clo[d] > chi[axis] - clo[axis]) { axis = d; }
    }
    const int mid = count / 2;
    auto centroid = [&](int tri) {
        return _v[_f[tri][0]][axis] + _v[_f[tri][1]][axis] +
               _v[_f[tri][2]][axis];
    };
    std::nth_element(_bvh_tri.begin() + start, _bvh_tri.begin() + start + mid,
        _bvh_tri.begin() + start + count,
        [&](int a, int b) { return centroid(a) < centroid(b); });
    const int l = buildBvh(start, mid);
    const int r = buildBvh(start + mid, count - mid);
    _bvh[id].left = l;
    _bvh[id].right = r;
    return id;
}

Smith2018DistanceField::Sample Smith2018DistanceField::sample(
    const Vec3& p) const
{
    double best = std::numeric_limits<double>::max();
    int best_tri = -1;
    Region best_region = FACE;
    Vec3 best_cp(0);

    int stack[128];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const BvhNode& nd = _bvh[stack[--sp]];
        if (boxDist2(p, nd.lo, nd.hi) >= best) { continue; }
        if (nd.left < 0) {
            for (int t = nd.start; t < nd.start + nd.count; ++t) {
                const int f = _bvh_tri[t];
                Region region;
                const Vec3 cp = closestPointOnTriangle(p, _v[_f[f][0]],
                    _v[_f[f][1]], _v[_f[f][2]], region);
                const Vec3 r = p - cp;
                const double d2 = SimTK::dot(r, r);
                if (d2 < best) {
                    best = d2; best_tri = f; best_region = region; best_cp = cp;
                }
            }
        }
        else {
            // the nearer child is visited first (pushed last)
            const double dl = boxDist2(p, _bvh[nd.left].lo, _bvh[nd.left].hi);
            const double dr = boxDist2(p, _bvh[nd.right].lo, _bvh[nd.right].hi);
            if (dl <= dr) { stack[sp++] = nd.right; stack[sp++] = nd.left; }
            else { stack[sp++] = nd.left; stack[sp++] = nd.right; }
        }
    }

    const int* v = _f[best_tri].data();
    Vec3 pn;
    bool rim = false;
    switch (best_region) {
    case FACE: pn = _face_normal[best_tri]; break;
    case VERT_A: pn = _vert_normal[v[0]]; rim = _vert_rim[v[0]] != 0; break;
    case VERT_B: pn = _vert_normal[v[1]]; rim = _vert_rim[v[1]] != 0; break;
    case VERT_C: pn = _vert_normal[v[2]]; rim = _vert_rim[v[2]] != 0; break;
    default: {
        const int e0 = (best_region == EDGE_AB) ? v[0] :
                       (best_region == EDGE_BC) ? v[1] : v[2];
        const int e1 = (best_region == EDGE_AB) ? v[1] :
                       (best_region == EDGE_BC) ? v[2] : v[0];
        const EdgeInfo& e = _edges.find(edgeKey(e0, e1))->second;
        pn = e.normal_sum;
        rim = (e.count == 1);
    }
    }
    const double dist = std::sqrt(best);
    // a point on the surface itself counts as outside
    const bool inside = SimTK::dot(p - best_cp, pn) < 0.0;
    Sample s;
    s.phi = inside ? -dist : dist;
    s.tri = best_tri;
    s.rim = rim;
    s.closest = best_cp;
    return s;
}

// 0 on a free edge of the mesh, 1 at the taper width or further away, smooth
// (smoothstep) in between.
double Smith2018DistanceField::edgeWeight(const Vec3& c) const
{
    if (!(_taper > 0.0) || _boundary_edges.empty()) { return 1.0; }
    double best = std::numeric_limits<double>::max();
    for (const auto& e : _boundary_edges) {
        const Vec3 ab = e[1] - e[0];
        const double len2 = SimTK::dot(ab, ab);
        double t = (len2 > 0) ? SimTK::dot(c - e[0], ab) / len2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        const Vec3 r = c - (e[0] + ab * t);
        best = std::min(best, SimTK::dot(r, r));
    }
    const double s = std::min(1.0, std::sqrt(best) / _taper);
    return s * s * (3.0 - 2.0 * s);
}

const Smith2018DistanceField::Block& Smith2018DistanceField::getBlock(
    int bi, int bj, int bk) const
{
    const size_t b = ((size_t)bk * _nb[1] + bj) * _nb[0] + bi;
    if (_block_ready[b].load(std::memory_order_acquire)) { return *_blocks[b]; }

    std::lock_guard<std::mutex> lock(_mutex);
    if (_block_ready[b].load(std::memory_order_acquire)) { return *_blocks[b]; }

    std::unique_ptr<Block> blk(new Block);
    for (int k = 0; k < B; ++k)
    for (int j = 0; j < B; ++j)
    for (int i = 0; i < B; ++i) {
        const int n = (k * B + j) * B + i;
        const Vec3 p = _origin + Vec3((bi * B + i) * _h, (bj * B + j) * _h,
                                      (bk * B + k) * _h);
        const Sample s = sample(p);
        blk->phi[n] = (float)s.phi;
        // without a taper, a node whose nearest point is on a free edge has
        // no contact (like a ray that misses the mesh)
        blk->weight[n] = (_taper > 0.0) ? (float)edgeWeight(s.closest) :
                                          (s.rim ? 0.0f : 1.0f);
        blk->tri[n] = s.tri;
        blk->rim[n] = s.rim ? 1 : 0;
        for (int d = 0; d < 3; ++d) {
            Vec3 dp(0);
            dp[d] = _h;
            blk->grad[n * 3 + d] =
                (float)((sample(p + dp).phi - sample(p - dp).phi) / (2 * _h));
        }
    }
    _num_computed += B * B * B;
    _blocks[b] = std::move(blk);
    _block_ready[b].store(1, std::memory_order_release);
    return *_blocks[b];
}

bool Smith2018DistanceField::evaluate(const Vec3& p, Query& q) const
{
    const Vec3 g = (p - _origin) / _h;
    int i0[3];
    double t[3];
    for (int d = 0; d < 3; ++d) {
        // range check in double before the int conversion (also rejects NaN)
        if (!(g[d] >= 0.0 && g[d] < double(_n[d] - 1))) { return false; }
        const double fl = std::floor(g[d]);
        i0[d] = (int)fl;
        t[d] = g[d] - fl;
    }

    double phi = 0.0;
    Vec3 grad(0);
    int nearest = -1;
    double edge_weight = 0.0;
    double nearest_node_weight = 0.0;
    double nearest_w = -1.0;
    for (int c = 0; c < 8; ++c) {
        const int di = c & 1, dj = (c >> 1) & 1, dk = (c >> 2) & 1;
        const int i = i0[0] + di, j = i0[1] + dj, k = i0[2] + dk;
        const Block& blk = getBlock(i / B, j / B, k / B);
        const int n = ((k % B) * B + (j % B)) * B + (i % B);
        const double w = (di ? t[0] : 1 - t[0]) * (dj ? t[1] : 1 - t[1]) *
                         (dk ? t[2] : 1 - t[2]);
        phi += w * blk.phi[n];
        grad += Vec3(blk.grad[n * 3], blk.grad[n * 3 + 1],
                     blk.grad[n * 3 + 2]) * w;
        edge_weight += w * blk.weight[n];
        if (w > nearest_w) {
            nearest_w = w; nearest = blk.tri[n]; nearest_node_weight = blk.weight[n];
        }
    }
    if (!(_taper > 0.0)) { edge_weight = nearest_node_weight; }

    if (!(phi < 0.0) || phi < -_band) { return false; }
    const double gn = grad.norm();
    if (!(gn > 1e-9)) { return false; }

    q.depth = -phi;
    q.outward = grad / gn;
    q.triangle = nearest;
    q.weight = edge_weight;
    return true;
}

} // namespace OpenSim
