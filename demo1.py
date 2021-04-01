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
import matplotlib.pyplot as plt

def main():
    runs = 1
    for r in range(0,runs):
        print("running "+ str(r))
  
    physicsClient = p.connect(p.DIRECT) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("/home/shiyani/stacking/plane_files/plane.urdf")
    maximalCoordinates = False
    count = 0
    z = 0.7
    x = 0
    y = 0

    robot = p.loadURDF("/home/shiyani/stacking/pb_robot/models/turtlebot/turtlebot.urdf",[0,.7,0],useMaximalCoordinates=maximalCoordinates)

    #adding the table
    t1 = p.loadURDF("/home/shiyani/stacking/pb_robot/models/table_collision/table.urdf",[0,0,0],useMaximalCoordinates=maximalCoordinates)

        #adding the cube using loadURDF
    while count != runs:
        
        cube = p.loadURDF("/home/shiyani/stacking/deform/src/geo/cube_10_6_6.urdf",[x,y,z],useMaximalCoordinates=maximalCoordinates,flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        z = z 
        x = x + .08
        y = y + .14
        count = count + 1

    #############################################################################################################################################################################

    
    
    ########################################################################################################################################################################################
    
    

    p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0,0,0])

    viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 3],
    cameraTargetPosition=[0, .5, 0],
    cameraUpVector=[0, 1, 0])

    projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

    # width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    # width=224, 
    # height=224,
    # viewMatrix=viewMatrix,
    # projectionMatrix=projectionMatrix,
    # renderer= p.ER_BULLET_HARDWARE_OPENGL)
    #renderer=p.ER_TINY_RENDERER)
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
    # rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    # seg_tiny = np.reshape(images[4], [width, height]) * 1. / 255.
    rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1
    seg_tiny = np.reshape(images[4], [width, height]) * 1. 
    

    #plt.subplot(4, 2, 6)
    plt.imshow(seg_tiny)
    plt.savefig("seg_image_seg_tiny.png")
    plt.title('Seg Tiny')
    #plt.subplots_adjust(hspace=0.7)

    ###################################################################################################################################################
    # # RGB Images
    # image_array = np.asarray(rgbImg)
    # image_rgb = Image.fromarray(image_array.astype('uint8'))
    # # Depth Images
    # image_array = np.asarray(depthImg)
    # image_depth = Image.fromarray(image_array.astype('uint8'))
    # # Seg Images
    #image_array = np.asarray(images)
    #image_seg = Image.fromarray(image_array.astype('uint8'))
    # # Saving Images
    #image_seg.save("/home/shiyani/stacking/Image/seg/image_%d.png" % r)
    # image_rgb.save("/home/shiyani/stacking/Image/rgb/image_%d.png" % r)
    # image_depth.save("/home/shiyani/stacking/Image/depth/image_%d.png" % r)

    #p.disconnet()
    time.sleep(15)
if __name__ == '__main__':
    
    # test_exploration(args)
    
    main()
