#include <pybind11/pybind11.h>
#include <collision_detector/collision_detector.hpp>
#include <Eigen/Core>

namespace py = pybind11;

PYBIND11_MODULE(pybind_collision_detector, m) {
  py::class_<CollisionDetector>(m, "CollisionDetector")
      .def(py::init<const std::string &>())
      .def("detectCollision", [](CollisionDetector &self, float x, float y,
                                 float z, float yaw, float pitch, float roll) {
        // Turn x, y, z, yaw, pitch, roll into a eigen transform
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(x, y, z);
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        transform.linear() = rotation;

        return self.detectCollision(transform);
      });
}
