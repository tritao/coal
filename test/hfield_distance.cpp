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

/// Distance between a height field and shapes or meshes: signed distances,
/// closest points and normals, for a flat and a sloped field, placed with an
/// arbitrary transform, in both argument orders.

#define BOOST_TEST_MODULE COAL_HEIGHT_FIELD_DISTANCE
#include <boost/test/included/unit_test.hpp>

#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "coal/hfield.h"
#include "coal/math/transform.h"
#include "coal/shape/convex.h"
#include "coal/shape/geometric_shapes.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace coal;

namespace {

const CoalScalar kTolerance = 1e-8;

/// A 2 m x 2 m field of 5 x 5 vertices (cells of 0.5 m), from -1 m up to
/// heights z = slope * x + offset.
template <typename BV>
std::shared_ptr<HeightField<BV> > field(CoalScalar slope, CoalScalar offset) {
  const Eigen::DenseIndex n = 5;
  MatrixXs heights(n, n);
  for (Eigen::DenseIndex row = 0; row < n; ++row)
    for (Eigen::DenseIndex col = 0; col < n; ++col)
      heights(row, col) = slope * (-1. + 0.5 * CoalScalar(col)) + offset;
  return std::make_shared<HeightField<BV> >(2., 2., heights, -1.);
}

/// An arbitrary placement for the field: the expected values are computed in
/// the field's frame and carried along.
Transform3s placement() {
  Transform3s tf;
  tf.setQuatRotation(Quatf(Eigen::AngleAxis<CoalScalar>(
      0.7, Vec3s(0.2, -0.4, 1.).normalized())));
  tf.setTranslation(Vec3s(0.3, -1.2, 0.5));
  return tf;
}

std::shared_ptr<Convex<Triangle> > cube(CoalScalar half) {
  std::shared_ptr<std::vector<Vec3s> > points(new std::vector<Vec3s>());
  for (int i = 0; i < 8; ++i)
    points->push_back(Vec3s(i & 1 ? half : -half, i & 2 ? half : -half,
                            i & 4 ? half : -half));
  std::shared_ptr<std::vector<Triangle> > triangles(
      new std::vector<Triangle>({Triangle(0, 2, 1), Triangle(1, 2, 3),
                                 Triangle(4, 5, 6), Triangle(5, 7, 6),
                                 Triangle(0, 1, 4), Triangle(1, 5, 4),
                                 Triangle(2, 6, 3), Triangle(3, 6, 7),
                                 Triangle(0, 4, 2), Triangle(2, 4, 6),
                                 Triangle(1, 3, 5), Triangle(3, 7, 5)}));
  return std::make_shared<Convex<Triangle> >(points, 8, triangles, 12);
}

std::shared_ptr<BVHModel<OBBRSS> > cube_mesh(CoalScalar half) {
  std::shared_ptr<Convex<Triangle> > solid = cube(half);
  std::vector<Vec3s> points(solid->points->begin(), solid->points->end());
  std::vector<Triangle> triangles(solid->polygons->begin(),
                                  solid->polygons->end());
  auto mesh = std::make_shared<BVHModel<OBBRSS> >();
  mesh->beginModel();
  mesh->addSubModel(points, triangles);
  mesh->endModel();
  return mesh;
}

/// Checks the distance (both argument orders), the closest points (the one
/// on the field lies on its surface z = slope * x + offset, in its frame)
/// and the normal, against an expected signed distance and normal in the
/// field's frame.
template <typename BV>
void check(const HeightField<BV>& hfield, CoalScalar slope, CoalScalar offset,
           const CollisionGeometry& shape, const Transform3s& shape_in_field,
           CoalScalar expected, const Vec3s& expected_normal) {
  const Transform3s tf1 = placement();
  const Transform3s tf2 = tf1 * shape_in_field;
  DistanceRequest request(true, true);
  {
    DistanceResult result;
    const CoalScalar d = distance(&hfield, tf1, &shape, tf2, request, result);
    BOOST_CHECK_CLOSE_FRACTION(d + 10., expected + 10., kTolerance);
    const Vec3s on_field = tf1.inverseTransform(result.nearest_points[0]);
    BOOST_CHECK_SMALL(on_field[2] - (slope * on_field[0] + offset), 1e-6);
    const Vec3s gap = result.nearest_points[1] - result.nearest_points[0];
    BOOST_CHECK_SMALL(gap.norm() - std::abs(d), 1e-6);
    BOOST_CHECK(
        (tf1.getRotation().transpose() * result.normal - expected_normal)
            .isZero(1e-6));
  }
  {
    DistanceResult result;
    const CoalScalar d = distance(&shape, tf2, &hfield, tf1, request, result);
    BOOST_CHECK_CLOSE_FRACTION(d + 10., expected + 10., kTolerance);
    const Vec3s on_field = tf1.inverseTransform(result.nearest_points[1]);
    BOOST_CHECK_SMALL(on_field[2] - (slope * on_field[0] + offset), 1e-6);
    BOOST_CHECK(
        (tf1.getRotation().transpose() * result.normal + expected_normal)
            .isZero(1e-6));
  }
}

Transform3s at(CoalScalar x, CoalScalar y, CoalScalar z) {
  return Transform3s(Matrix3s::Identity(), Vec3s(x, y, z));
}

template <typename BV>
void flat_field() {
  auto hfield = field<BV>(0., 0.2);
  const Vec3s up(0., 0., 1.);
  const Sphere sphere(0.1);
  // Clear, then penetrating from above near a cell's centre (the top face is
  // the nearest face of its prism).
  check(*hfield, 0., 0.2, sphere, at(0.25, 0.25, 0.7), 0.4, up);
  check(*hfield, 0., 0.2, sphere, at(0.25, 0.25, 0.25), -0.05, up);
  check(*hfield, 0., 0.2, Box(0.2, 0.4, 0.6), at(-0.4, 0.3, 1.), 0.5, up);
  check(*hfield, 0., 0.2, Box(0.2, 0.2, 0.2), at(-0.25, -0.25, 0.25), -0.05,
        up);
  check(*hfield, 0., 0.2, Capsule(0.05, 0.4), at(0.6, -0.6, 0.6), 0.15, up);
  check(*hfield, 0., 0.2, *cube(0.1), at(0.1, 0.4, 0.5), 0.2, up);
  check(*hfield, 0., 0.2, *cube_mesh(0.1), at(0.1, 0.4, 0.5), 0.2, up);

  // A swept-sphere radius shrinks the distance by exactly that much.
  Sphere inflated(0.1);
  inflated.setSweptSphereRadius(0.05);
  check(*hfield, 0., 0.2, inflated, at(0.25, 0.25, 0.7), 0.35, up);
}

template <typename BV>
void sloped_field() {
  // z = 0.2 x: each prism's top lies in that plane.
  const CoalScalar slope = 0.2;
  auto hfield = field<BV>(slope, 0.);
  const Vec3s normal = Vec3s(-slope, 0., 1.).normalized();
  const CoalScalar scale = std::sqrt(1. + slope * slope);
  // A sphere: its centre's distance to the plane, less the radius.
  check(*hfield, slope, 0., Sphere(0.1), at(0.1, 0.2, 0.6),
        (0.6 - slope * 0.1) / scale - 0.1, normal);
  // A box: its lowest corner against the plane is (+half, *, -half).
  check(*hfield, slope, 0., Box(0.2, 0.2, 0.2), at(0., 0., 0.8),
        (0.7 - slope * 0.1) / scale, normal);
  check(*hfield, slope, 0., *cube(0.1), at(0., 0., 0.8),
        (0.7 - slope * 0.1) / scale, normal);
  check(*hfield, slope, 0., *cube_mesh(0.1), at(0., 0., 0.8),
        (0.7 - slope * 0.1) / scale, normal);
}

}  // namespace

BOOST_AUTO_TEST_CASE(hfield_distance_flat) {
  flat_field<AABB>();
  flat_field<OBBRSS>();
}

BOOST_AUTO_TEST_CASE(hfield_distance_sloped) {
  sloped_field<AABB>();
  sloped_field<OBBRSS>();
}

BOOST_AUTO_TEST_CASE(hfield_distance_agrees_with_collision) {
  auto hfield = field<OBBRSS>(0.2, 0.1);
  const Transform3s tf1 = placement();
  const Sphere sphere(0.1);
  DistanceRequest distance_request(true, true);
  CollisionRequest collision_request(CONTACT, 1);
  for (int i = 0; i < 40; ++i) {
    const CoalScalar z = -0.1 + 0.02 * CoalScalar(i);
    const Transform3s tf2 = tf1 * at(0.3, -0.2, z);
    DistanceResult distance_result;
    const CoalScalar d = distance(&*hfield, tf1, &sphere, tf2,
                                  distance_request, distance_result);
    CollisionResult collision_result;
    collide(&*hfield, tf1, &sphere, tf2, collision_request, collision_result);
    if (std::abs(d) > 1e-6)
      BOOST_CHECK_EQUAL(d < 0, collision_result.isCollision());
  }
}

BOOST_AUTO_TEST_CASE(hfield_distance_far_shape_skips_no_cell) {
  // Beside the field: its side face at x = 1 is nearest.
  auto hfield = field<OBBRSS>(0., 0.);
  const Sphere sphere(0.1);
  DistanceRequest request(true, true);
  DistanceResult result;
  const CoalScalar d = distance(&*hfield, Transform3s::Identity(), &sphere,
                                at(4., 0., 0.), request, result);
  BOOST_CHECK_CLOSE_FRACTION(d, 2.9, kTolerance);
  BOOST_CHECK_SMALL(result.nearest_points[0][0] - 1., 1e-6);
}
