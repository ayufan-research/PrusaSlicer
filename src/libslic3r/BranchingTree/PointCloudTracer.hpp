#ifndef POINTCLOUDTRACER_HPP
#define POINTCLOUDTRACER_HPP

#include "PointCloud.hpp"
#include "libslic3r/AStar.hpp"
#include "libslic3r/SLA/SupportTreeUtils.hpp"

namespace Slic3r {

struct PointCloudTracer {
    using Node = branchingtree::Node;
    const branchingtree::PointCloud &pc;
    const AABBMesh &mesh;
    mutable int id_from = 0;

    template<class Fn>
    void foreach_reachable(const Node &from, Fn &&fn) const
    {
        using namespace sla;

        //        Vec3f p = from.pos;
        Vec3d fromd = from.pos.cast<double>();
        //        pc.foreach_reachable(p, [this, &fn, &from, &fromd](size_t node_id, float /*distance*/) {
        //            if (pc.get_type(node_id) == vanektree::BED) {
        //                const Node &node = pc.get(node_id);
        //                bool free = !beam_mesh_hit(ex_tbb, mesh,
        //                              Beam{Ball{fromd, from.Rmin},
        //                                   Ball{node.pos.cast<double>(), node.Rmin}},
        //                              0.5).is_hit();

        //                if (free) fn(node);
        //            }
        //        });

        constexpr size_t N = 8;
        PointRing<N> ring{Vec3d{0.1, 0.1, -1.0}.normalized()};

        Vec3d ringsrc = fromd;
        ringsrc.z() -= 2.;
        double ringR = 2.;
        bool goal_reached = false;

        Vec3d nodep = ringsrc;
        branchingtree::Node node(nodep.cast<float>(), from.Rmin);

        bool free = !beam_mesh_hit(ex_tbb, mesh,
                                   Beam{Ball{fromd, from.Rmin},
                                        Ball{nodep, node.Rmin}},
                                   0.5).is_hit();

        if (free) {
            node.id = id_from++;
            goal_reached = fn(node);
        }

        for (size_t n = 0; n < N && !goal_reached; ++n) {
            Vec3d nodep = ring.get(n, ringsrc, ringR);
            branchingtree::Node node(nodep.cast<float>(), from.Rmin);

            bool free = !beam_mesh_hit(ex_tbb, mesh,
                                       Beam{Ball{fromd, from.Rmin},
                                            Ball{nodep, node.Rmin}},
                                       0.5).is_hit();

            if (free) {
                node.id = id_from++;
                goal_reached = fn(node);
            }
        }
    }

    float distance(const Node &a, const Node &b) const
    {
        return (a.pos - b.pos).squaredNorm(); //pc.get_distance(a.pos, b.id);
    }

    float goal_heuristic(const Node & n) const
    {
        const Node *bednode = pc.closest_bedpt(n.pos);
        double dist = std::numeric_limits<float>::infinity();

        if (n.pos.z() < pc.properties().ground_level()) {
            dist = -1.f;
        }
        else if (bednode)
            dist = (n.pos - bednode->pos).squaredNorm();

        return dist;
    }

    size_t unique_id(const Node &n) const { return n.id; }
};

} // namespace Slic3r

#endif // POINTCLOUDTRACER_HPP
