#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <iostream>
#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[]) {
  std::cout << "pcd2bt convert a pcd file into octree representation and save "
               "it in a bt file."
            << std::endl;

  // read the first argument as a pcd file
  if (argc <= 1) {
    std::cout << "usage: " << argv[0] << " <pointcloud.pcd>" << std::endl;
    exit(0);
  }

  // read in the pointcloud using pcl library
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    PCL_ERROR("Couldn't read file %s\n", argv[1]);
    return (-1);
  }

  // print the number of points in the pointcloud
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from "
            << argv[1] << std::endl;

  // create an octomap pointcloud object and copy all points
  const auto [min, max] = std::minmax_element(
      cloud->points.begin(), cloud->points.end(),
      [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) { return a.z < b.z; });
  auto z_offset = (max->z + min->z) / 2;
  auto z_range = (max->z - min->z);
  auto resolution = z_range / 64 * 1.01;

  octomap::Pointcloud octomap_cloud;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    octomap_cloud.push_back(cloud->points[i].x, cloud->points[i].y,
                            cloud->points[i].z - z_offset);
  }

  // write the pointcloud to the octree
  auto tree = new octomap::OcTree(resolution);
  tree->insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0));
  tree->updateInnerOccupancy();

  // print the tree depth
  std::cout << "Tree depth: " << tree->getTreeDepth() << std::endl;

  for (auto it = tree->begin_tree(tree->getTreeDepth()), end = tree->end_tree();
       it != end; ++it) {

    if (it.isLeaf()) {
      // std::cout << "Node center: " << it.getCoordinate() << " " <<
      // it->getValue() << std::endl;
    }
    // std::cout << it.getKey()[0] << " " << it.getKey()[1] << " " <<
    // it.getKey()[2] << " " << std::endl;
  }
  // save the octree to a binary file
  std::string filename = argv[1];
  filename = filename.replace(filename.end() - 4, filename.end(), ".bt");
  tree->writeBinary(filename);
  std::cout << "Saved " << filename << std::endl;

  // create a new octree object in fcl from the octree object in octomap

  auto tree_ptr = std::shared_ptr<const octomap::OcTree>(tree);
  auto tree_fcl = std::make_shared<fcl::OcTree<double>>(tree_ptr);

  // create a collision object in fcl from the octree object
  fcl::CollisionObject<double> tree_obj(tree_fcl, fcl::Transform3<double>());

  // create a box object in fcl
  auto box = std::make_shared<fcl::Box<double>>(0.05, 0.05, 0.05);

  // create a collision object in fcl from the box object
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  auto start = std::chrono::high_resolution_clock::now();
  bool res = false;
  int repeat = 1000;
  fcl::CollisionObject<double> box_obj(box, transform);

  // create a collision request object
  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;

  for (size_t i = 0; i < 1000; i++) {
    transform.translation() = Eigen::Vector3d(rand(), rand(), rand());
    box_obj.setTransform(transform);

    // check for collision between the octree object and the box object
    fcl::collide<double>(&tree_obj, &box_obj, request, result);

    res = result.isCollision();
  }

  // Get the ending time point
  auto end = std::chrono::high_resolution_clock::now();

  // Calculate the duration
  std::chrono::duration<double> duration = end - start;

  std::cout << "Time taken by function: " << duration.count() / repeat * 1000 << " ms"
            << std::endl;

  // print the result of the collision check
  if (res) {
    std::cout << "Collision detected" << std::endl;
  } else {
    std::cout << "No collision detected" << std::endl;
  }

  return 0;
}