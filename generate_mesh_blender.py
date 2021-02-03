import bpy
import numpy as np
import bmesh
import mathutils
from mathutils import Vector
import math
import os
import json


def point_cloud(ob_name, coords, edges=[], faces=[]):
    """Create point cloud object based on given coordinates and name.

    Keyword arguments:
    ob_name -- new object name
    coords -- float triplets eg: [(-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)]
    """

    # Create new mesh and a new object
    me = bpy.data.meshes.new(ob_name + "Mesh")
    ob = bpy.data.objects.new(ob_name, me)

    # Make a mesh from a list of vertices/edges/faces
    me.from_pydata(coords, edges, faces)

    # Display name and update the mesh
    ob.show_name = True
    me.update()
    
    # Link object to the active collection
    bpy.context.collection.objects.link(ob)

    return ob

def to_convex_hull(ob):
    bm = bmesh.new()
    me = ob.data
    bm.from_mesh(me)
    bmesh.ops.convex_hull(bm, input=bm.verts)
    bm.to_mesh(me)
    me.update()
    bm.clear()


def get_elliptope_vertices(N=10):
    """
    Vertices on the boundary of 
    {(x,y,z) | [[1, x, y], [x, 1, z], [y, z, 1]] psd}
    """
    
    
    thetas = np.linspace(0, 2*np.pi, N)
    phis = np.linspace(0, np.pi, N)
    to_cart = lambda theta, phi: np.array([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta)])
        
    I = np.eye(3)
    
    vertices = [(1,1,1), (-1,-1,-1)]
    for theta in thetas:
        for phi in phis:
            # direction in (x, y, z) space
            d = to_cart(theta, phi)
            x,y,z = d
            # direction in matrix space
            D = np.array([[0, x, y], [x, 0, z], [y, z, 0]])
            
            # max gamma s.t. I + gamma D psd
            gamma = 1. / max(np.linalg.eigvals(D))
            
            vertices.append(gamma * d)
    return vertices
            



# objective function

def make_arrow(loc, dir):
    
    v2 = Vector(dir)

    # duplicate mesh into new object.
    arrow_mesh = bpy.data.objects['Arrow_model'].data
    obj = bpy.data.objects.new("Arrow_duplicate", arrow_mesh) 

    #obj.scale = (.1, .1, 1)
    obj.location = v2
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = v2.to_track_quat('Z','Y')
    obj.location = Vector(loc)
    
    bpy.context.collection.objects.link(obj)
    
def make_plane_normal_to(name, dist, norm):
    v1, v2 = Vector([0,0,0]), Vector(norm)
    
    # create a new cube
    bpy.ops.mesh.primitive_plane_add()

    # newly created cube will be automatically selected
    plane = bpy.context.selected_objects[0]
    plane.name = name
    plane.rotation_mode = 'QUATERNION'
    plane.rotation_quaternion = (v1-v2).to_track_quat('Z', 'Y')
    plane.scale = (2,2,2)
    plane.location += dist * v2
    return plane
    
    
def create_transparent_mat(name, color, alpha):
    material = bpy.data.materials.new(name)
    material.use_nodes = True

    mat_properties = material.node_tree.nodes["Principled BSDF"].inputs
    mat_alpha = mat_properties[18]
    mat_alpha.default_value = alpha
    mat_color = mat_properties[0]
    mat_color.default_value = [*color, 1.]
    return material
    
    
def cylinder_between(x1, y1, z1, x2, y2, z2, r):
  dx = x2 - x1
  dy = y2 - y1
  dz = z2 - z1    
  dist = math.sqrt(dx**2 + dy**2 + dz**2)

  bpy.ops.mesh.primitive_cylinder_add(
      radius = r, 
      depth = dist,
      location = (dx/2 + x1, dy/2 + y1, dz/2 + z1)   
  ) 
  cylinder =  bpy.context.object
  phi = math.atan2(dy, dx) 
  theta = math.acos(dz/dist) 

  cylinder.rotation_euler[1] = theta 
  cylinder.rotation_euler[2] = phi 
  cylinder.name = 'Cylinder'
  return cylinder



# Create elliptope
pc = point_cloud("Elliptope", get_elliptope_vertices(N=100))
to_convex_hull(pc)



c = [-0.25, 0.0, -0.25]
optimal_sol = [-1, 1, -1]
dist = 4
make_arrow(optimal_sol, c)
make_plane_normal_to('Normal_Plane', dist, c)
A_z = make_plane_normal_to('A_z', dist, c)


# colors

red_transparent = create_transparent_mat("red_transparent", (1, 0, 0), .8)
bpy.data.objects['Elliptope'].data.materials.append(red_transparent)

blue_mat = create_transparent_mat("red_transparent", (1, 1, 0.1), 1)
bpy.data.objects['Normal_Plane'].data.materials.append(blue_mat)
bpy.data.objects['A_z'].data.materials.append(blue_mat)

# animation

# load data

# load data
filepath = bpy.data.filepath
directory = os.path.dirname(filepath)
data_file = os.path.join( directory , "renegar_output.json")
json_data = json.load(open(data_file, "r"))
z_data = json_data['z'][::5]


# animation
scene = bpy.context.scene
number_frames = len(z_data)
scene.frame_end = len(z_data)
for i, z in enumerate(z_data):
    scene.frame_set(i)
    
    A_z.location = np.array(c) * z / np.linalg.norm(c)**2
    A_z.keyframe_insert(data_path="location", index=-1)