import open3d as o3d
import numpy as np
import json

print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("./point_cloud.pcd")

print(pcd)
print(np.asarray(pcd.points))
# o3d.visualization.draw(pcd)

pts = np.asarray(pcd.points)
pts_max = np.max(pts, axis=0)
pts_min = np.min(pts, axis=0)
pts_range = pts_max - pts_min


centre = (pts_max + pts_min) / 2
centre[0] = 0
centre[1] = 0
centre[2] = 0
print(centre)
sides = 0.4
half_sides = sides / 2

cropped = {
    "axis_max": 1,
    "axis_min": -0.6,
    "class_name": "SelectionPolygonVolume",
    "orthogonal_axis": "Z",
    "bounding_polygon": [
        list(centre + [half_sides, half_sides, 0]),
        list(centre + [-half_sides, half_sides, 0]),
        list(centre + [-half_sides, -half_sides, 0]),
        list(centre + [half_sides, -half_sides, 0]),
    ],
    "version_major": 1,
    "version_minor": 0,
}

import json

with open("cropped_rect.json", "w") as f:
    f.write(json.dumps(cropped))

print(
    centre,
    list(centre + [half_sides, half_sides, 0]),
    list(centre + [-half_sides, half_sides, 0]),
)

vol = o3d.visualization.read_selection_polygon_volume("cropped_rect.json")
cropped_pcd = vol.crop_point_cloud(pcd)


pts = np.asarray(cropped_pcd.points)
print(pts)
pts_max = np.max(pts, axis=0)
pts_min = np.min(pts, axis=0)
pts_range = pts_max - pts_min

print(pts_max, pts_min, pts_range)

# o3d.visualization.draw(cropped_pcd)


def bits(f):
    bytes = f.read()
    for b in bytes:
        for i in range(8):
            yield (b >> i) & 1


class Node:
    FREE = 0
    OCCUPIED = 1
    CHILDREN = 2
    UNDEFINED = 3

    def __init__(self, status):
        if status[0] == 1 and status[1] == 0:
            # child is a free leaf
            self.status = Node.FREE
        elif status[0] == 0 and status[1] == 1:
            # child is a occupied leaf
            self.status = Node.OCCUPIED
        elif status[0] == 1 and status[1] == 1:
            # child has children
            self.status = Node.CHILDREN
        else:
            self.status = Node.UNDEFINED

        self.children = []

    def has_children(self):
        return self.status == Node.CHILDREN

    def add_child(self, node):
        self.children.append(node)


def add_children(node, nodes, idx):
    for i in range(8):
        child = Node(nodes[idx + i])
        node.add_child(child)

    idx += 8

    children = node.children
    for i, child in enumerate(children):
        if child.has_children():
            child, idx = add_children(child, nodes, idx)
            node.children[i] = child

    return node, idx


class BinaryOctree:
    def __init__(self, fn="cropped_point_cloud.bt"):
        self.nodes = []

        with open(fn, "br") as f:
            for i in range(3):
                header = f.readline()
                print(header)
            id = f.readline().split()[-1]
            size = int(f.readline().split()[-1])
            res = float(f.readline().split()[-1])
            print(f"id {id}, size {size}, res {res}")
            data_header = f.readline()

            self.data = bits(f)

            node = []
            for idx, bit in enumerate(self.data):
                if idx % 2 == 0:
                    node = [bit]
                else:
                    node.append(bit)
                    self.nodes.append(node)

        root = Node([1, 1])
        idx = 0
        root, idx = add_children(root, self.nodes, idx)

        def print_tree(parent):
            print(parent)
            for child in parent.children:
                print_tree(child)

        # print_tree(root)

        self.root = root
        print(self.nodes[:16])

    def serialize(self, fn):
        max_depth = 8
        dflt_color = [0.45490196078431372, 0.47843137254901963, 0.32549019607843138]

        def serialize_node(node, depth):
            s = {}
            if node.has_children():
                # if depth < max_depth:
                s["class_name"] = "OctreeInternalNode"
                s["children"] = []
                for child in node.children:
                    s["children"].append(serialize_node(child, depth + 1))

                c = s["children"]
                # s["children"] = list(reversed(c))
                # s["children"] = [ c[1], c[0], c[3], c[2],  c[5], c[4],  c[7], c[6] ]
                # else:
                #     s["class_name"] = "OctreeColorLeafNode"
                #     s["color"] = dflt_color
            elif node.status == Node.OCCUPIED:
                s["class_name"] = "OctreeColorLeafNode"
                s["color"] = dflt_color
            return s

        depth = 1
        s = serialize_node(self.root, depth)

        print("self.max_depth()", self.max_depth())
        ss = {
            "class_name": "Octree",
            "max_depth": self.max_depth(),
            "origin": [-1.9999353885650635, -1.9999524354934692, -3.079195961356163],
            "size": 4.0399324548244477,
            "tree": s,
        }

        with open(fn, "w") as f:
            json.dump(ss, f, indent=4)
        # for node in self.nodes:
        #     print(node, len(self.nodes))

    def max_depth(self):
        def max_depth(parent):
            if not parent.has_children():
                return 0
            return 1 + max([max_depth(child) for child in parent.children])

        return max_depth(self.root)

    def squash(self):
        still_squashing = True
        while still_squashing:
            children_children_idx = []
            for child in self.root.children:
                children_children_status = [c.status for c in child.children]
                print(child.status, children_children_status)
                if child.status == Node.FREE or child.status == Node.OCCUPIED:
                    still_squashing = False
                    break
                elif child.status == Node.CHILDREN:
                    if (
                        children_children_status.count(Node.FREE)
                        + children_children_status.count(Node.UNDEFINED)
                        == 7
                    ):
                        children_children_idx.append(children_children_status.index(Node.CHILDREN))
                    else:
                        still_squashing = False
                        break
                elif child.status == Node.UNDEFINED:
                    children_children_idx.append(8)

            print(children_children_idx)
            can_squash = True if len(children_children_idx) == 8 else False
            for i, idx in enumerate(children_children_idx):
                if not (idx == 7 - i or idx == 8):
                    can_squash = False
                    break

            if (can_squash):
                for i in range(8):
                    if self.root.children[i].status == Node.CHILDREN:
                        self.root.children[i] = self.root.children[i].children[7 - i]
            else:
                still_squashing = False

        # for i in range(7):
        #     for i in range(8):
        #         if self.root.children[i].status == Node.CHILDREN:
        #             self.root.children[i] = self.root.children[i].children[7 - i]


bo = BinaryOctree()
bo.squash()
bo.serialize("octree_dumped.json")

# load octree from json
octree = o3d.geometry.Octree(max_depth=bo.max_depth())
octree = o3d.io.read_octree("octree_dumped.json")


print("octree division")
# octree = o3d.geometry.Octree(max_depth=7)
# octree.convert_from_point_cloud(cropped_pcd, size_expand=0.01)
# cropped_pcd = o3d.io.read_point_cloud("./cropped_point_cloud.pcd")
# o3d.io.write_point_cloud("cropped_point_cloud.pcd", cropped_pcd)

o3d.visualization.draw([cropped_pcd, octree])
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(octree)
# vis.get_render_option().mesh_show_wireframe = False
# vis.run()
# vis.destroy_window()
# print("run Poisson surface reconstruction")
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=9
#     )
# print(mesh)
# o3d.visualization.draw_geometries(
#     [mesh],
#     zoom=0.664,
#     front=[-0.4761, -0.4698, -0.7434],
#     lookat=[1.8900, 3.2596, 0.9284],
#     up=[0.2304, -0.8825, 0.4101],
# )
