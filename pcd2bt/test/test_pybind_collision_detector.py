if __name__ == "__main__":
    from pybind_collision_detector import CollisionDetector

    c = CollisionDetector("../../cropped_point_cloud.pcd")

    assert(c.detectCollision(0.0, 0.0, 0.0, 0,0,0) == True)
    assert(c.detectCollision(1, 1, 1, 0,0,0) == False)


