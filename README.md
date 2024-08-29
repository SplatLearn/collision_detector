# Collision Detector Between Point Cloud and Bounding Box

## Build

Install Dependencies

```sh
> sudo apt install build-essential libpcl-dev liboctomap-dev libeigen3-dev pybind11-dev libfcl-dev libccd-dev
```

Build the project using cmake

```sh
> mkdir build
> cd build
> cmake ..
> make -j
```

## Usage

The compilation process produces file called `pybind_collision_detector.*.so`. This file need to be copied to be within PYTHONPATH. Then from python:

```python
from nerfgym.pybind_collision_detector import (
    CollisionDetector,
    CollisionDetectorOptions,
)
```

To detect collision between a point cloud and bounding box:

```python
opt = CollisionDetectorOptions()

# specify parameter of the bounding box
bbox_sides = 1
opt.x_max = bbox_sides / 2
opt.x_min = -bbox_sides / 2
opt.y_max = bbox_sides / 2
opt.y_min = -bbox_sides / 2
opt.z_max = 1
opt.z_min = -0.6
pcd_file = "..."

c = CollisionDetector(pcd_file, opt)

# detect collision specifying x, y, z, roll, pitch, yaw of the bounding box
r = c.detectCollision(x, y, z, roll, pitch, yaw)
if r:
    print("collision detected")
```
