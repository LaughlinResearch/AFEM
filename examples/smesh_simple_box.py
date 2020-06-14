from afem.geometry import Point
from afem.graphics import Viewer
from afem.smesh import (MeshGen, NetgenAlgo2D, NetgenSimple2D, MaxLength1D,
                        Regular1D)
from afem.topology import *

# Create a simple solid box
box = BoxBySize(10, 10, 10).solid

# Get a list of faces and edges of the shape for later use
faces = box.faces
edges = box.edges

# Initialize the mesh generator
gen = MeshGen()

# Create a new mesh using the box as the shape
mesh = gen.create_mesh(box)

# Define algorithms and hypotheses
alg2d = NetgenAlgo2D(gen)
hyp2d_1 = NetgenSimple2D(gen, 1.)
hyp2d_2 = NetgenSimple2D(gen, 1., allow_quads=False)
alg1d = Regular1D(gen)
hyp1d = MaxLength1D(gen, 0.25)

# Add them to the mesh
mesh.add_hypotheses([alg2d, hyp2d_1])
mesh.add_hypothesis(hyp2d_2, faces[-1])
mesh.add_hypotheses([alg1d, hyp1d], edges[-1])

# Compute the mesh
gen.compute(mesh)

# View the mesh
gui = Viewer()
gui.add(mesh)
gui.start()
gui.clear()

# Get sub-meshes from sub-shapes. Note that extracting the nodes and elements
# from a shape is better done with groups.
face_submesh = mesh.get_submesh(faces[0])
edge_submesh = mesh.get_submesh(edges[0])

# View the face sub-mesh (2-D elements)
gui.add(face_submesh)
gui.start()
gui.clear()

# View the edge sub-mesh (1-D elements)
gui.add(edge_submesh)
gui.start()

# Use groups to organize mesh data
edge_nodes = mesh.create_group('edge nodes', mesh.NODE, edges[-1])
face_nodes = mesh.create_group('face nodes', mesh.NODE, faces[-1])

# Show edge nodes
gui.clear()
gui.add(mesh, edge_nodes)
gui.start()

# Show face nodes
gui.clear()
gui.add(mesh, face_nodes)
gui.start()

# Subtract the edge nodes from the face nodes. Other group methods include
# union and intersect.
group = face_nodes.subtract(edge_nodes)
gui.clear()
gui.add(mesh, group)
gui.start()
