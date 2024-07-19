#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <iostream>

int main(int argc, char *argv[]) {
  double w = 0.05;
  double d = 0.05;
  double h = 0.05;

  double w2 = 0.4;
  double d2 = 0.4;
  double h2 = 0.4;

  auto box_geometry = std::make_shared<fcl::Box<double>>(w, d, h);
  auto box2_geometry = std::make_shared<fcl::Box<double>>(w2, d2, h2);
  fcl::Transform3<double> X_WB = fcl::Transform3<double>::Identity();
  fcl::Transform3<double> X_WB2 = fcl::Transform3<double>::Identity();
  X_WB2.translation() = fcl::Vector3<double>(0.0277778, 0.0277778, 0.0833333);
  fcl::CollisionObject<double> box(box_geometry, X_WB);
  fcl::CollisionObject<double> box2(box2_geometry, X_WB2);

  fcl::CollisionRequest<double> collision_request(1 /* num contacts */,
                                                  true /* enable_contact */);
  fcl::CollisionResult<double> collision_result;

  std::size_t contact_count =
      fcl::collide(&box, &box2, collision_request, collision_result);

  std::cout << "Number of contacts: " << contact_count << std::endl;

  return 0;
}