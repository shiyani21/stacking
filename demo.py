import pdb
from collections import defaultdict, deque, namedtuple
import numpy as np
import os
import random
import glob
import pybullet as p
import pb_robot.helper as helper
from pybullet_utils import PyBulletServer, quat_math
import pybullet_data
import time
from PIL import Image
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def sort(array,array1,var):
    n = len(array)

    for i in range(n):
        # Create a flag that will allow the function to
        # terminate early if there's nothing left to sort
        already_sorted = True
        if var == "x_pos":

        # Start looking at each item of the list one by one,
        # comparing it with its adjacent value. With each
        # iteration, the portion of the array that you look at
        # shrinks because the remaining items have already been
        # sorted.
            for j in range(n - i - 1):
                #if array[j] == array[j+1] and array[j] + dim_x < x_boundary_right and array[j] + dim_x < x_boundary_left:
                if array[j] == array[j+1] and array1[j] <= stack_lim :
                    # If the item you're looking at is greater than its
                    # adjacent value, then swap them
                    # array[j] = random.uniform(-0.65,.75)
                    #if the x value of array[j] is the same as the one next to it then I want to stack the blocks, so 
                    array1[j] = array1[j]+dim_z
                if array[j] == array[j+1] and array1[j] > stack_lim :
                    #if the stack is already high enough 
                    array[j] = random.uniform(-0.65,.75)
                if array[j] + dim_x < array[j+1] and array[j] - dim_x > array[j+1] and array[j] + dim_x < x_boundary_right and array[j] + dim_x < x_boundary_left:
                    #if the array[j] block is within the upper and lower lim of the 2nd block, just pick one at random
                    array[j] = random.uniform(-0.65,.75)
                    #array[j] = array[j] + dim_x
                    
                    
                    already_sorted = False

            # If there were no swaps during the last iteration,
            # the array is already sorted, and you can terminate
            if already_sorted:
                break

    return array

def get_transform(view_mat,projection_mat,model_mat,vertex_points,transfrom_list):
    pv = np.matmul(projection_mat,view_mat) 
    mvp = np.matmul(pv,model_mat)
    vertex_points = np.array(vertex_points)
    val = vertex_points.shape[0]
    add_1 = np.ones((val,1))
    vertex_points =np.append(vertex_points, add_1, axis=1)
        
    for i in range(len(vertex_points)):
        
        transfrom = np.matmul(mvp,vertex_points[i])
        transfrom_list.append(transfrom)

    return transfrom_list

def read_obj(path,decompose=True):
    mesh = Mesh([], [])
    meshes = {}
    vertices = []
    faces = []
    for line in helper.read(path).split('\n'):
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == 'o':
            name = tokens[1]
            mesh = Mesh([], [])
            meshes[name] = mesh
        elif tokens[0] == 'v':
            vertex = tuple(map(float, tokens[1:4]))
            vertices.append(vertex)
        elif tokens[0] in ('vn', 's'):
            pass
        elif tokens[0] == 'f':
            face = tuple(int(token.split('/')[0]) - 1 for token in tokens[1:])
            faces.append(face)
            mesh.faces.append(face)
    if not decompose:
        return Mesh(vertices, faces)
    for name, mesh in meshes.items():
        indices = sorted({i for face in mesh.faces for i in face})
        mesh.vertices[:] = [vertices[i] for i in indices]
        new_index_from_old = {i2: i1 for i1, i2 in enumerate(indices)}
        mesh.faces[:] = [tuple(new_index_from_old[i1] for i1 in face) for face in mesh.faces]
    
    return meshes,vertices,faces


def transform_obj_file(obj_string, transformation):
    new_lines = []
    for line in obj_string.split('\n'):
        tokens = line.split()
        if not tokens or (tokens[0] != 'v'):
            new_lines.append(line)
            continue
        vertex = list(map(float, tokens[1:]))
        transformed_vertex = transformation.dot(vertex)
        new_lines.append('v {}'.format(' '.join(map(str, transformed_vertex))))
    return '\n'.join(new_lines)



def main():
    global x_boundary_right
    global x_boundary_left
    global y_boundary_up
    global y_boundary_down
    global dim_x 
    global dim_y 
    global dim_z
    global stack_lim
    global Mesh 
    p.connect(p.DIRECT) 
    #p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    #p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("/home/shiyani/stacking/plane_files/plane.urdf")
    maximalCoordinates = False
    
    t1 = p.loadURDF("/home/shiyani/stacking/pb_robot/models/table_collision/table.urdf",[0,0,0],useMaximalCoordinates=maximalCoordinates)

    array_z = []
    array_x = []
    array_y = []

    tf_list =[]

    x_prev = 0
    y_prev = 0
    z_pos = .69
    x_boundary_right = .75
    x_boundary_left = -.65
    y_boundary_up = 0.4
    y_boundary_down = -.45
    dim_x = .08
    dim_y = .14
    dim_z = .12
    stack_lim = .93
    x = "x_pos"
    y = "y_pos"
    Mesh = namedtuple('Mesh', ['vertices', 'faces'])

    #number of runs
    for i in range(2):
        x_pos=random.uniform(-0.65,.75) 
        y_pos=random.uniform(-0.45,0.4)
        z_pos = .69
        array_x.append(x_pos)
        array_y.append(y_pos)
        array_z.append(z_pos)
    final_x_array = sort(array_x,array_z,x)
    
    robot = p.loadURDF("/home/shiyani/stacking/pb_robot/models/turtlebot/turtlebot.urdf",[0,1,.5],useMaximalCoordinates=maximalCoordinates)
   

    

    for i in range(len(final_x_array)):
        random_int = random.randrange(-314,314)/100
        face_pos_pitch =[1.5708,3.1415,-15.708,-3.1415]#this for the pitch
        face_pos_roll = [1.5708,3.1415,-1.5708]
        random_pitch = random.randint(0,3)
        random_roll = random.randint(0,2)
        cubeStartPos = [final_x_array[i],array_y[i],array_z[i]]
        cubeStartOrientation = p.getQuaternionFromEuler([face_pos_pitch[random_pitch],face_pos_roll[random_roll],random_int])
        cube = p.loadURDF("/home/shiyani/stacking/deform/src/geo/cube_obj/cube_10_6_6.urdf",cubeStartPos,cubeStartOrientation,useMaximalCoordinates=maximalCoordinates,flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        path = "/home/shiyani/stacking/deform/src/geo/cube_obj/cube_10_6_6.obj"
        list_mesh = read_obj(path, decompose=True)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(cube)
        euler = p.getEulerFromQuaternion(cubeOrn)
        rot = R.from_quat(cubeStartOrientation)
        rot_mat = rot.as_matrix()
        rot_mat = rot_mat.round(decimals=2, out=None)
        translation = np.array(cubeStartPos).reshape(3,1)
        extra = np.array([[0, 0, 0, 1]], dtype=np.float32)
        model = np.concatenate((np.concatenate((rot_mat, translation), axis=1),extra),axis =0)
        #np.concatenate( (np.concatenate((R1, T1), axis=1), a), axis=0 ) 
        #print(model)

        #print(rot_mat)
       
  
       

    #############################################################################################################################################################################

    
    
    ########################################################################################################################################################################################

        p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0,0,0])
        viewMatrix = p.computeViewMatrix(
        cameraEyePosition=[0, -1.75, 1.75],
        cameraTargetPosition=[0, 0, 0],
        cameraUpVector=[0, 1, 0])

        projectionMatrix = p.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=1.0,
        nearVal=0.1,
        farVal=3.1)
        
        #transformation stuff

        #print("this is the view space: ")
        
        view_new = np.reshape(viewMatrix,(4,4))
        view = view_new.transpose()
        #print(view)
        #print("this is the projection matrix:")
        projection_new =np.reshape(projectionMatrix,(4,4))
       
        projection = projection_new.transpose()
        projection = projection_new
        #print(projection_new)
        #focal_length = 
        #print("the transformation shoudl look like")
        
        #i have no clue what i am doingc
        meshes,vertices_list,faces = read_obj(path,decompose=True)
        #print(vertices_list)
        # vertices_list = np.array(vertices_list)

        transform = get_transform(view,projection,model,vertices_list,tf_list)
        for i in range(len(transform)):
            print("the transfroms for vertices are",transform[i])

        #######################################################################################################################################

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=224, 
        height=224,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        renderer= p.ER_TINY_RENDERER)
    

        images = p.getCameraImage(width = 224,
                        height = 224,
                        viewMatrix=viewMatrix,
                        projectionMatrix=projectionMatrix,
                        flags=p.ER_USE_PROJECTIVE_TEXTURE,
                        projectiveTextureView=viewMatrix,
                        projectiveTextureProj=projectionMatrix,
                        shadow=True,
                        renderer=p.ER_TINY_RENDERER)
    
    #i have no clue what i am doing
        width = 224
        height = 224
        nearVal=0.1
        farVal=3.1
        
        depth_buffer_tiny = np.reshape(images[3], [width, height])
        depth_tiny = farVal * nearVal / (farVal - (farVal - nearVal) * depth_buffer_tiny)
    
        rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1
        seg_tiny = np.reshape(images[4], [width, height]) * 1. 

        #plt show for rgb image
        plt.imshow(rgb_tiny)
        plt.savefig("/home/shiyani/stacking/Image/rgb/image_%d.png" % i)
        plt.title('rgb Tiny')
        
        #plt show for depth image
        plt.imshow(depth_tiny)
        plt.savefig("/home/shiyani/stacking/Image/depth/image_%d.png" % i)
        plt.title('Depth Tiny')

        #plt show for seg image
        plt.imshow(seg_tiny)
        plt.savefig("/home/shiyani/stacking/Image/seg/image_%d.png" % i)
        plt.title('Seg Tiny')
    #renderer=p.ER_TINY_RENDERER)
    
    ###################################################################################################################################################

    time.sleep(10)
if __name__ == '__main__':
    
    # test_exploration(args)
    
    main()
