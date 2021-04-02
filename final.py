import pybullet as p
import time
import pybullet_data
import random

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
            
                    already_sorted = False

            # If there were no swaps during the last iteration,
            # the array is already sorted, and you can terminate
            if already_sorted:
                break

    return array


def main():
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    plane = p.loadURDF("/home/shiyani/stacking/plane_files/plane.urdf")

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


    for i in range(4):
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
        cube = p.loadURDF("/home/shiyani/stacking/models/Cube_Wired.urdf",cubeStartPos,cubeStartOrientation)
        #cube = p.loadURDF("cube_10_6_6.urdf",cubeStartPos, cubeStartOrientation)
        print(p.getCollisionShapeData(cube, linkIndex = -1))

        collision_shapes = p.getCollisionShapeData(cube,-1)
        for shape_index in range (len(collision_shapes)):
            shape = collision_shapes[shape_index]
            mesh = p.getMeshData(cube,-1,shape_index)
            num_verts = mesh[0]
            vertices=mesh[1]
            print("num_verts=",num_verts)
            print("vertices=",vertices)

