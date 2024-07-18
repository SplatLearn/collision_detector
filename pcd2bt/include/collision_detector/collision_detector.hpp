#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <iostream>
#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

// Detect collision between a point cloud and a bounding box
class CollisionDetector {
public:
  CollisionDetector(std::string pcd_file) {
    // Initialize the collision detector

    // read in the pointcloud using pcl library
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
      PCL_ERROR("Couldn't read file %s\n", pcd_file.c_str());
      throw std::runtime_error("Couldn't read file");
    }

    // print the number of points in the pointcloud
    std::cout << "Loaded " << cloud->width * cloud->height
              << " data points from " << pcd_file << std::endl;

    // create an octomap pointcloud object and copy all points
    const auto [min, max] =
        std::minmax_element(cloud->points.begin(), cloud->points.end(),
                            [](const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
                              return a.z < b.z;
                            });

    // calculate the z offset and resolution to make sure the points fill the
    // octree boxes
    auto z_offset = (max->z + min->z) / 2;
    auto z_range = (max->z - min->z);
    auto resolution =
        z_range / 64 * 1.01; // inflate the bbox to cover all points

    // Convert pcl point cloud into an octomap point cloud
    octomap::Pointcloud octomap_cloud;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      octomap_cloud.push_back(cloud->points[i].x, cloud->points[i].y,
                              cloud->points[i].z - z_offset);
    }

    // write the pointcloud to the octree
    tree_ = std::make_shared<octomap::OcTree>(resolution);
    tree_->insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0));
    tree_->updateInnerOccupancy();

    // save the octree to a binary file
    std::string filename = pcd_file;
    filename = filename.replace(filename.end() - 4, filename.end(), ".bt");
    tree_->writeBinary(filename);
    std::cout << "Saved " << filename << std::endl;

    // create a new octree object in fcl from the octree object in octomap
    tree_fcl_ = std::make_shared<fcl::OcTree<double>>(tree_);

    // create a collision object in fcl from the octree object
    tree_obj_ = std::make_shared<fcl::CollisionObject<double>>(
        tree_fcl_, fcl::Transform3<double>());

    // create a box object in fcl
    auto box = std::make_shared<fcl::Box<double>>(0.05, 0.05, 0.05);

    // create a collision object in fcl from the box object
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    box_obj_ = std::make_shared<fcl::CollisionObject<double>>(box, transform);
  }

  bool detectCollision(fcl::Transform3<double> transform) {
    // Detect collision between the point cloud and the bounding box
    box_obj_->setTransform(transform);

    // check for collision between the octree object and the box object
    fcl::collide<double>(&*tree_obj_, &*box_obj_, collision_request_,
                         collision_result_);

    return collision_result_.isCollision();
  }

private:
  // create a collision request object
  fcl::CollisionRequest<double> collision_request_;
  fcl::CollisionResult<double> collision_result_;

  // the objects for collision detection
  std::shared_ptr<octomap::OcTree> tree_{nullptr};
  std::shared_ptr<fcl::OcTree<double>> tree_fcl_{nullptr};
  std::shared_ptr<fcl::CollisionObject<double>> tree_obj_{nullptr};
  std::shared_ptr<fcl::CollisionObject<double>> box_obj_{nullptr};
};