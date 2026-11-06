#ifndef OPENSIM_SMITH2018_DISTANCE_FIELD_H_
#define OPENSIM_SMITH2018_DISTANCE_FIELD_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  Smith2018DistanceField.h                     *
 * -------------------------------------------------------------------------- *
 * Internal helper of Smith2018ArticularContactForce (contact_method           *
 * "distance_field"). Not part of the public API.                              *
 * -------------------------------------------------------------------------- */

#include <SimTKcommon.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace OpenSim {

/** Signed distance field of a triangle mesh on a regular grid (in the frame of
the mesh).

The sign follows the outward triangle normals (negative = behind the surface,
i.e. inside a closed mesh or beneath an open articular surface), found with
angle-weighted pseudo-normals, so it also works for open meshes. The distance
and its gradient (the direction of the contact force) are stored on the grid
nodes and interpolated trilinearly; the nearest triangle gives the material
properties.

The nodes are computed on demand, in blocks of 8x8x8, from a bounding volume
hierarchy of the triangles, so only the region where the other mesh actually
comes close is ever evaluated. evaluate() is thread safe.

Points further than negative_band behind the surface or in front of the
surface report "no contact". Near a free edge of an open mesh the query reports
a weight that goes smoothly from 1 (edge_taper_width or more from the edge) to 0
(at the edge), so that the contact force of a triangle that slides off the edge
fades out instead of dropping to zero abruptly (which stops the integrator). */
class Smith2018DistanceField {
public:
    Smith2018DistanceField(
        const SimTK::Vector_<SimTK::Vec3>& vertices,
        const std::vector<std::vector<int>>& faces,
        double voxel_size, double negative_band, double edge_taper_width);

    struct Query {
        double depth;          // > 0 behind the surface (= -signed distance)
        SimTK::Vec3 outward;   // unit gradient of the signed distance
        int triangle;          // nearest triangle
        double weight;         // 0 at a free edge of the mesh, 1 beyond the
                               // taper width (smooth in between)
    };

    /** Evaluate at a point in the mesh frame. Returns false if the point is
    outside of the band, beside the rim, or in front of the surface. */
    bool evaluate(const SimTK::Vec3& p, Query& q) const;

    /** Number of grid nodes computed so far. */
    long long getNumComputedNodes() const { return _num_computed; }
    bool isClosed() const { return _num_boundary_edges == 0; }
    int getNumBoundaryEdges() const { return _num_boundary_edges; }
    double getVoxelSize() const { return _h; }

private:
    static const int B = 8; // nodes per block edge

    struct Block {
        float phi[B * B * B];
        float weight[B * B * B];
        float grad[B * B * B * 3];
        int tri[B * B * B];
        unsigned char rim[B * B * B];
    };
    struct BvhNode {
        SimTK::Vec3 lo, hi;
        int left = -1, right = -1; // children, or -1
        int start = 0, count = 0;  // triangle range of a leaf
    };
    struct EdgeInfo {
        SimTK::Vec3 normal_sum = SimTK::Vec3(0);
        int count = 0;
    };
    struct Sample {
        double phi;
        int tri;
        bool rim;
        SimTK::Vec3 closest;
    };

    int buildBvh(int start, int count);
    Sample sample(const SimTK::Vec3& p) const;
    double edgeWeight(const SimTK::Vec3& closest_point) const;
    const Block& getBlock(int bi, int bj, int bk) const;

    SimTK::Vec3 _origin;
    double _h;
    double _band;
    double _taper;
    std::vector<std::array<SimTK::Vec3, 2>> _boundary_edges; // free edges
    int _n[3];  // nodes per direction
    int _nb[3]; // blocks per direction
    int _num_boundary_edges = 0;

    std::vector<SimTK::Vec3> _v;
    std::vector<std::array<int, 3>> _f;
    std::vector<SimTK::Vec3> _face_normal;
    std::vector<SimTK::Vec3> _vert_normal;
    std::vector<unsigned char> _vert_rim;
    std::unordered_map<std::uint64_t, EdgeInfo> _edges;

    std::vector<BvhNode> _bvh;
    std::vector<int> _bvh_tri; // triangle order of the leaves

    mutable std::vector<std::unique_ptr<Block>> _blocks;
    mutable std::vector<std::atomic<unsigned char>> _block_ready;
    mutable std::mutex _mutex;
    mutable std::atomic<long long> _num_computed{0};
};

} // namespace OpenSim

#endif
