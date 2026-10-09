/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of INRIA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/// A convex set given by its points alone (no neighbour graph) with more
/// points than the large-convex threshold: GJK must fall back to the linear
/// support scan instead of dereferencing the missing neighbours.

#define BOOST_TEST_MODULE COAL_CONVEX_WITHOUT_NEIGHBORS
#include <boost/test/included/unit_test.hpp>

#include "coal/collision.h"
#include "coal/distance.h"
#include "coal/shape/geometric_shapes.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace coal;

namespace {

/// A convex set from points only, as a caller without a hull builder makes it.
class PointsOnly : public ConvexBase {
 public:
  explicit PointsOnly(std::shared_ptr<std::vector<Vec3s> > points) {
    const unsigned int count = static_cast<unsigned int>(points->size());
    set(points, count);
    computeLocalAABB();
  }
  PointsOnly* clone() const override { return new PointsOnly(*this); }

 private:
  PointsOnly(const PointsOnly& other) = default;
};

/// Points on a sphere of radius 0.5, more than the large-convex threshold.
std::shared_ptr<std::vector<Vec3s> > sphere_points(size_t count) {
  std::shared_ptr<std::vector<Vec3s> > points(new std::vector<Vec3s>());
  const CoalScalar golden = M_PI * (3. - std::sqrt(5.));
  for (size_t i = 0; i < count; ++i) {
    const CoalScalar z = 1. - 2. * (CoalScalar(i) + 0.5) / CoalScalar(count);
    const CoalScalar r = std::sqrt(1. - z * z), a = golden * CoalScalar(i);
    points->push_back(0.5 * Vec3s(r * std::cos(a), r * std::sin(a), z));
  }
  return points;
}

}  // namespace

BOOST_AUTO_TEST_CASE(convex_without_neighbors_above_threshold) {
  const size_t count = 2 * ConvexBase::num_vertices_large_convex_threshold;
  std::shared_ptr<std::vector<Vec3s> > points = sphere_points(count);
  PointsOnly convex(points);
  BOOST_CHECK(convex.neighbors.get() == nullptr);
  CoalScalar max_x = -1;
  for (const Vec3s& p : *points) max_x = std::max(max_x, p[0]);

  // A box whose face x = 1 faces the convex set: the distance is 1 - max x.
  const Box box(1., 10., 10.);
  const Transform3s box_pose(Matrix3s::Identity(), Vec3s(1.5, 0., 0.));
  DistanceRequest request(true, true);
  DistanceResult result;
  const CoalScalar d =
      distance(&convex, Transform3s::Identity(), &box, box_pose, request, result);
  BOOST_CHECK_SMALL(d - (1. - max_x), 1e-8);

  // Against another points-only set two above: at least the gap between their
  // z extents, at most the closest pair of vertices.
  std::shared_ptr<std::vector<Vec3s> > other_points = sphere_points(count + 3);
  PointsOnly other(other_points);
  const Vec3s lift(0., 0., 2.);
  CoalScalar top = -1, bottom = 1, closest_vertices = 1e9;
  for (const Vec3s& p : *points) top = std::max(top, p[2]);
  for (const Vec3s& q : *other_points) {
    bottom = std::min(bottom, q[2]);
    for (const Vec3s& p : *points)
      closest_vertices = std::min(closest_vertices, (q + lift - p).norm());
  }
  result.clear();
  const CoalScalar between =
      distance(&convex, Transform3s::Identity(), &other,
               Transform3s(Matrix3s::Identity(), lift), request, result);
  BOOST_CHECK(between >= 2. + bottom - top - 1e-9);
  BOOST_CHECK(between <= closest_vertices + 1e-9);
  CollisionRequest collision_request(CONTACT, 1);
  CollisionResult collision_result;
  collide(&convex, Transform3s::Identity(), &other,
          Transform3s(Matrix3s::Identity(), Vec3s(0., 0., 0.5)),
          collision_request, collision_result);
  BOOST_CHECK(collision_result.isCollision());
}
