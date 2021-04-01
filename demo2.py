import pdb
#import argparse
import numpy as np
#import matplotlib.pyplot as plt
#sjp
import os
import random
import glob
import pybullet as p
from pybullet_utils import PyBulletServer, quat_math
import pybullet_data
import time
from PIL import Image

def main():
    runs = int(input("How many runs?: "))
    if runs > 0:
        runs = runs
    else:
        runs = 3

    array_x =[]
    array_y = []
    array_z = []

    boundary_x_right, boundary_x_left, boundary_y_up, boundary_y_down, boundary_z =.75,-.65,.4,-.45,0.69

    dim_x,dim_y,dim_z = .08,.14,.12

    count = 0
    while count != runs:
        print("running "+ str(count))

        pos_x, pos_y, pos_z = [float(k) for k in input("position of cube in the order x,y,z : ").split(",")]
    

        if count == 0:
            array_x.append(pos_x)
            array_y.append(pos_y)
            array_z.append(pos_z)
        else:
            for i in array_z:
                if array_x[i] == pos_x or array_y[i] == pos_y or array_z[i] < boundary_z:
                    pos_z = pos_z+dim_z
                if array_x[i] == pos_x or array_x[i] <= (pos_x - dim_x) or array_x[i] > boundary_x_right or array_x[i] > boundary_x_left:
                    pos_x = pos_x + dim_x
                if array_y[i] == pos_y or array_y[i] <= (pos_y - dim_y) or array_y[i] > boundary_y_up or array_y[i] > boundary_y_down:
                    pos_y = pos_y + dim_y
        count= count+1

    #this is using the GUI    
    physicsClient = p.connect(p.DIRECT) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("/home/shiyani/stacking/plane_files/plane.urdf")
    maximalCoordinates = False
    # count = 0
    # z = 0.7
    # x = 0
    # y = 0

    robot = p.loadURDF("/home/shiyani/stacking/pb_robot/models/turtlebot/turtlebot.urdf",[0,.7,0],useMaximalCoordinates=maximalCoordinates)

    #adding the table
    t1 = p.loadURDF("/home/shiyani/stacking/pb_robot/models/table_collision/table.urdf",[0,0,0],useMaximalCoordinates=maximalCoordinates)

    

    #adding the cube using loadURDF
    # while count != runs:
    for i in range(len(array_x)):
        # we are still figuring out the loop!!!
        print("what is x[i] : ",type(array_x[i]))
        print("what is y[i] : ",array_y[i])
        print("what is z[i] :", array_z[i])
        
        cube = p.loadURDF("/home/shiyani/stacking/deform/src/geo/cube_10_6_6.urdf",[array_x[i],array_y[i],array_z[i]],useMaximalCoordinates=maximalCoordinates,flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        print("the cube has been loaded")
        #count = count + 1

    #############################################################################################################################################################################

    
    
    ########################################################################################################################################################################################
        p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=30, cameraPitch=52, cameraTargetPosition=[0,0,0])

        viewMatrix = p.computeViewMatrix(
        cameraEyePosition=[0, 0, 3],
        cameraTargetPosition=[0, .5, 0],
        cameraUpVector=[0, 1, 0])

        projectionMatrix = p.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=1.0,
        nearVal=0.1,
        farVal=3.1)

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=224, 
        height=224,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        renderer= p.ER_BULLET_HARDWARE_OPENGL)
        
    #renderer=p.ER_TINY_RENDERER)
    
    ###################################################################################################################################################
        
    # RGB Images
        image_array = np.asarray(rgbImg)
        image_rgb = Image.fromarray(image_array.astype('uint8'))
        # Depth Images
        image_array = np.asarray(depthImg)
        image_depth = Image.fromarray(image_array.astype('uint8'))
        # Seg Images
        image_array = np.asarray(segImg)
        image_seg = Image.fromarray(image_array.astype('uint8'))
        # Saving Images
        image_seg.save("/home/shiyani/stacking/Image/seg/image_%d.png" % i)
        print("printing seg image!")
        image_rgb.save("/home/shiyani/stacking/Image/rgb/image_%d.png" % i)
        image_depth.save("/home/shiyani/stacking/Image/depth/image_%d.png" % i)

    #p.disconnet()
    time.sleep(15)
if __name__ == '__main__':
    
    # test_exploration(args)
    
    main()
