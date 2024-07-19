#include <pybind11/pybind11.h>
#include <collision_detector/collision_detector.hpp>
#include <Eigen/Core>

namespace py = pybind11;

PYBIND11_MODULE(pybind_collision_detector, m) {
  py::class_<CollisionDetector>(m, "CollisionDetector")
      .def(py::init<const std::string &, const CollisionDetectorOptions>())
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


  // bind CollisionDetectorOptions struct to python
  py::class_<CollisionDetectorOptions>(m, "CollisionDetectorOptions")
      .def(py::init<>())
      .def_readwrite("x_min", &CollisionDetectorOptions::x_min)
      .def_readwrite("x_max", &CollisionDetectorOptions::x_max)
      .def_readwrite("y_min", &CollisionDetectorOptions::y_min)
      .def_readwrite("y_max", &CollisionDetectorOptions::y_max)
      .def_readwrite("z_min", &CollisionDetectorOptions::z_min)
      .def_readwrite("z_max", &CollisionDetectorOptions::z_max);
}
