if __name__ == "__main__":
    from pybind_collision_detector import CollisionDetector, CollisionDetectorOptions
    import numpy as np
    import matplotlib.pyplot as plt

    opt = CollisionDetectorOptions()

    bbox_sides = 1
    opt.x_max = bbox_sides / 2
    opt.x_min = -bbox_sides / 2
    opt.y_max = bbox_sides / 2
    opt.y_min = -bbox_sides / 2
    opt.z_max = 1
    opt.z_min = -0.6

    c = CollisionDetector("../../point_cloud.pcd", opt)

    n_steps = 10
    x_ = np.linspace(-0.25, 0.25, n_steps)
    y_ = np.linspace(-0.25, 0.25, n_steps)
    z_ = np.linspace(-0.6, -0.1, n_steps)

    x, y, z = np.meshgrid(x_, y_, z_, indexing="ij")

    result = np.ones([n_steps, n_steps, n_steps])

    pts = []
    colors = []

    for i in range(n_steps):
        for j in range(n_steps):
            for k in range(n_steps):
                r = c.detectCollision(x[i, j, k], y[i, j, k], z[i, j, k], 0, 0, 0)
                pts.append([x[i, j, k], y[i, j, k], z[i, j, k]])
                if r:
                    # print("collision at", x[i,j,k], y[i,j,k], z[i,j,k])
                    result[i, j, k] = 1
                    colors.append("red")
                else:
                    result[i, j, k] = 0
                    colors.append("blue")

    # print(result)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    # Voxels is used to customizations of the
    # sizes, positions and colors.
    ax.voxels(result, facecolors="blue", edgecolors="grey")
    # pts = np.array(pts)
    # ax.scatter(pts[:,0], pts[:,1], pts[:,2], c=colors, s=1)
    plt.show()

    assert c.detectCollision(0, 0, -0.5, 0, 0, 0) == True
    assert c.detectCollision(0, 0.4, -0.6, 0, 0, 0) == True
    assert c.detectCollision(1, 1, 0.6, 0, 0, 0) == False
