#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <iostream>
#include <octomap/octomap.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

struct CollisionDetectorOptions {
  double x_min{std::numeric_limits<double>::lowest()};
  double x_max{std::numeric_limits<double>::max()};
  double y_min{std::numeric_limits<double>::lowest()};
  double y_max{std::numeric_limits<double>::max()};
  double z_min{std::numeric_limits<double>::lowest()};
  double z_max{std::numeric_limits<double>::max()};
};

// Detect collision between a point cloud and a bounding box
class CollisionDetector {
public:
  CollisionDetector(const std::string pcd_file,
                    const CollisionDetectorOptions options = {}) {
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

    // crop the pointcloud to the specified bounding box
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloud);
    crop_filter.setMin(
        Eigen::Vector4f(options.x_min, options.y_min, options.z_min, 1.0));
    crop_filter.setMax(
        Eigen::Vector4f(options.x_max, options.y_max, options.z_max, 1.0));
    pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
    crop_filter.filter(cropped_cloud);

    // write the cropped pointcloud to a pcd file
    std::string filename = pcd_file;
    filename =
        filename.replace(filename.end() - 4, filename.end(), "_cropped.pcd");
    pcl::io::savePCDFileASCII(filename, cropped_cloud);

    // find the min and max z values in the cropped pointcloud
    Eigen::Vector4f min, max;
    pcl::getMinMax3D<pcl::PointXYZ>(cropped_cloud, min, max);
    auto range = max - min;
    auto max_range = std::max({range.x(), range.y(), range.z()});

    // calculate the resolution so it fills the octree boxes
    auto resolution =
        max_range / 64 * 1.01; // inflate the bbox to cover all points

    offset_ = (max + min) / 2;
    std::cout << "max: " << max << std::endl;
    std::cout << "min: " << min << std::endl;
    std::cout << "resolution: " << resolution << std::endl;
    std::cout << "offset: " << offset_ << std::endl;

    // create an octomap pointcloud object and copy all points
    octomap::Pointcloud octomap_cloud;
    for (size_t i = 0; i < cropped_cloud.points.size(); i++) {
      octomap_cloud.push_back(cropped_cloud.points[i].x,
                              cropped_cloud.points[i].y,
                              cropped_cloud.points[i].z);
    }

    // write the pointcloud to the octree
    tree_ = std::make_shared<octomap::OcTree>(resolution);
    tree_->insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0));
    tree_->updateInnerOccupancy();

    // save the octree to a binary file
    filename = pcd_file;
    filename = filename.replace(filename.end() - 4, filename.end(), ".bt");
    tree_->writeBinary(filename);
    std::cout << "Saved " << filename << std::endl;

    // create a new octree object in fcl from the octree object in octomap
    tree_fcl_ = std::make_shared<fcl::OcTree<double>>(tree_);

    // create a collision object in fcl from the octree object
    tree_obj_ = std::make_shared<fcl::CollisionObject<double>>(
        tree_fcl_, Eigen::Isometry3d::Identity());

    // create a box object in fcl
    box_ =
        std::make_shared<fcl::Box<double>>(resolution, resolution, resolution);

    // create a collision object in fcl from the box object
    box_obj_ = std::make_shared<fcl::CollisionObject<double>>(
        box_, Eigen::Isometry3d::Identity());
  }

  bool detectCollision(fcl::Transform3<double> transform) {
    // set the transform of the box object
    fcl::Transform3<double> box_transform = fcl::Transform3<double>::Identity();
    box_transform.translation() = transform.translation();
    box_obj_->setTransform(box_transform);

    // create a collision request object
    fcl::CollisionRequest<double> collision_request(1 /* num contacts */,
                                                    true /* enable_contact */);
    fcl::CollisionResult<double> collision_result;

    // check for collision between the octree object and the box object
    size_t num_contacts = fcl::collide<double>(
        box_obj_.get(), tree_obj_.get(), collision_request, collision_result);

    return num_contacts > 0;
  }

private:
  // the offset to the center of the point cloud
  Eigen::Vector4f offset_{0, 0, 0, 0};

  // the objects for collision detection
  std::shared_ptr<fcl::Box<double>> box_{nullptr};
  std::shared_ptr<octomap::OcTree> tree_{nullptr};
  std::shared_ptr<fcl::OcTree<double>> tree_fcl_{nullptr};
  std::shared_ptr<fcl::CollisionObject<double>> tree_obj_{nullptr};
  std::shared_ptr<fcl::CollisionObject<double>> box_obj_{nullptr};
};