#include <collision_detector/collision_detector.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[]) {
  std::cout << "pcd2bt convert a pcd file into octree representation and save "
               "it in a bt file."
            << std::endl;

  // read the first argument as a pcd file
  if (argc <= 1) {
    std::cout << "usage: " << argv[0] << " <pointcloud.pcd>" << std::endl;
    exit(0);
  }

  // create a collision detector object
  CollisionDetector detector(argv[1]);

  // Run collision detection repeatedly giving random bbox pose
  auto transform = fcl::Transform3<double>();
  const int repeat = 1000;
  bool res = false;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < repeat; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    // set the transform to a random pose
    transform.linear() = Eigen::Matrix3d::Identity();
    transform.translation() = Eigen::Vector3d(rand(), rand(), rand());
    res = detector.detectCollision(transform);
  }
  auto end = std::chrono::high_resolution_clock::now();

  // Calculate the time of each iteration
  std::chrono::duration<double> duration = end - start;
  std::cout << "Time taken by function: " << duration.count() / repeat * 1000
            << " ms" << std::endl;

  return 0;
}