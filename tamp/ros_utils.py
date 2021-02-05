"""
Utilities for ROS serialization and deserialization of task planning entities
"""

import pb_robot
from stacking_ros.msg import (
    BodyInfo, GoalInfo, RobotConfig, TaskAction, 
    TaskPlanGoal, TaskPlanResult, TrajInfo)
from tf.transformations import (
    quaternion_matrix, quaternion_from_matrix, translation_from_matrix)


############
# X to ROS #
############
def goal_to_ros(init, goal, fixed_objs):
    """ Convert PDDLStream planning initial conditions and goal specifications to a ROS Action Goal """
    ros_goal = TaskPlanGoal()
    print(init)
    print(goal)

    # Convert the PDDL Goal specification
    for elem in goal:
        if isinstance(elem, tuple):
            info = GoalInfo()
            info.type = elem[0]
            if info.type == "AtPose":    # e.g. ("AtPose", block1, pose)
                info.target_obj = elem[1].readableName
                pose_to_ros(elem[2], info.pose)
            elif info.type == "On":      # e.g. ("On", block1, block3)
                info.target_obj = elem[1].readableName
                info.base_obj = elem[2].readableName
            ros_goal.goal.append(info)
    
    # Convert the PDDL initial conditions (+ fixed objects)
    init_dict = {}
    init_rel_poses = []
    for elem in init:
        name = elem[0]                
        # Robot configuration e.g. ("Conf", conf)
        if name == "Conf":
            ros_goal.robot_config.angles = elem[1].configuration
        # Block pose information
        # Consists of sequence of: Graspable, Pose, AtPose, Block, On, Supported
        elif name in ["AtPose", "On"]:
            obj_name = elem[1].readableName
            if "block" in obj_name:
                if name == "AtPose":
                    init_dict[obj_name] = {"pose": elem[2]}
                elif name == "On":
                    init_dict[obj_name]["base_obj"] = elem[2].readableName
        # Relative pose information
        elif name == "RelPose":
            ros_blk = BodyInfo()
            ros_blk.is_rel_pose = True
            ros_blk.name = elem[1].readableName
            ros_blk.base_obj = elem[2].readableName
            transform_to_ros(elem[3], ros_blk.pose)
            init_rel_poses.append(ros_blk)
            
    for blk_name in init_dict:
        info = BodyInfo()
        info.name = blk_name
        pose_to_ros(init_dict[blk_name]["pose"], info.pose)
        for fobj in fixed_objs:
            if fobj is not None and blk_name == fobj.readableName:
                info.fixed = True
        ros_goal.blocks.append(info)
    ros_goal.blocks.extend(init_rel_poses)
    print(ros_goal)
    return ros_goal


def pose_to_ros(body_pose, msg):
    """ Passes BodyPose object data to ROS message """
    msg.position.x, msg.position.y, msg.position.z = body_pose.pose[0]
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = body_pose.pose[1]


def transform_to_ros(T, msg):
    """ Passes transformation matrix data to ROS message """
    msg.position.x, msg.position.y, msg.position.z = translation_from_matrix(T)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = \
        quaternion_from_matrix(T)


def grasp_to_ros(grasp, msg):
    """ Sets information from a BodyGrasp object in a ROS pose message """
    T = grasp.grasp_objF
    t = translation_from_matrix(T)
    msg.position.x, msg.position.y, msg.position.z = t
    q = quaternion_from_matrix(T)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q


def traj_to_ros(traj_list, msg):
    """ Sets trajectory information in a ROS TrajectoryInfo[] message """

    for traj in traj_list:
        traj_msg = TrajInfo()
        
        # Joint path case
        if (isinstance(traj, pb_robot.vobj.JointSpacePath)):
            traj_msg.type = "JointSpacePath"
            for config in traj.path:
                traj_msg.joint_path.append(
                    RobotConfig(angles=config))
            traj_msg.joint_path_speed = traj.speed
        # BodyGrasp case
        elif (isinstance(traj, pb_robot.vobj.BodyGrasp)):
            traj_msg.type = "BodyGrasp"
        # MoveToTouch case
        elif (isinstance(traj, pb_robot.vobj.MoveToTouch)):
            traj_msg.type = "MoveToTouch"
            traj_msg.use_wrist_camera = traj.use_wrist_camera
            traj_msg.q_start.angles = traj.start
            traj_msg.q_end.angles = traj.end
        # MoveFromTouch case
        elif (isinstance(traj, pb_robot.vobj.MoveFromTouch)):
            traj_msg.type = "MoveFromTouch"
            traj_msg.q_end.angles = traj.end

        msg.append(traj_msg)


def task_plan_to_ros(plan):
    """ Packs task plan information into a TaskPlanResult ROS message """
    result = TaskPlanResult()
    result.success = (plan is not None)
    if not result.success:
        return result

    for action in plan:
        act_type, act_args = action
        ros_act = TaskAction()
        ros_act.type = act_type

        # Action specific parameters
        if ros_act.type == "move_free":
            q1, q2, traj = act_args
        elif ros_act.type == "move_holding":
            q1, q2, obj1, grasp, traj = act_args
            ros_act.obj1 = obj1.readableName
            grasp_to_ros(grasp, ros_act.grasp)
        elif ros_act.type == "pick":
            obj1, pose1, obj2, grasp, q1, q2, traj = act_args
            ros_act.obj1 = obj1.readableName
            ros_act.obj2 = obj2.readableName
            pose_to_ros(pose1, ros_act.pose1)
            grasp_to_ros(grasp, ros_act.grasp)
        elif ros_act.type == "place":
            obj1, pose1, obj2, pose2, grasp, q1, q2, traj = act_args
            ros_act.obj1 = obj1.readableName
            ros_act.obj2 = obj2.readableName
            pose_to_ros(pose1, ros_act.pose1)
            pose_to_ros(pose2, ros_act.pose2)
            grasp_to_ros(grasp, ros_act.grasp)

        # Generic parameters
        ros_act.q1.angles = q1.configuration
        ros_act.q2.angles = q2.configuration
        traj_to_ros(traj, ros_act.trajectories)

        result.plan.append(ros_act)
    return result


############
# ROS to X #
############
def ros_to_pose(msg, body):
    """ Creates a BodyPose from a ROS message """
    p = [msg.position.x, msg.position.y, msg.position.z]
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    return pb_robot.vobj.BodyPose(body, (p,q))


def ros_to_transform(msg):
    """ Extracts a pose from a ROS message """
    p = [msg.position.x, msg.position.y, msg.position.z]
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    T = quaternion_matrix(q)
    T[0:3,-1] = p
    return T


def ros_to_body_config(msg, robot):
    """ Convert ROS configuration message to BodyConfig """
    return pb_robot.vobj.BodyConf(robot, msg.angles)


def ros_to_traj(msg, robot, pddl_block_lookup):
    """ Gets trajectory information from a ROS TaskPlan message """
    traj_list = []
    for ros_traj in msg.trajectories:
        name = ros_traj.type
        if name == "JointSpacePath":
            path = [q.angles for q in ros_traj.joint_path]
            speed = ros_traj.joint_path_speed
            traj = pb_robot.vobj.JointSpacePath(
                robot.arm, path, speed)
        # BodyGrasp case
        elif name == "BodyGrasp":
            body = pddl_block_lookup[msg.obj1]
            T = ros_to_transform(msg.grasp)
            traj = pb_robot.vobj.BodyGrasp(body, T, robot.arm)
        # MoveToTouch case
        elif name == "MoveToTouch":
            q_approach = ros_traj.q_start.angles
            q_grasp = ros_traj.q_end.angles
            body = pddl_block_lookup[msg.obj1]
            T = ros_to_transform(msg.grasp)
            grasp = pb_robot.vobj.BodyGrasp(body, T, robot.arm)
            traj = pb_robot.vobj.MoveToTouch(
                robot.arm, q_approach, q_grasp, grasp, body, 
                ros_traj.use_wrist_camera)
        # MoveFromTouch case
        elif name == "MoveFromTouch":
            q_end = ros_traj.q_end.angles 
            traj = pb_robot.vobj.MoveFromTouch(
                robot.arm, q_end)

        traj_list.append(traj)
    return traj_list


def ros_to_task_plan(msg, robot, pddl_block_lookup):
    """ Unpacks task plan information from a TaskPlanResult ROS message """
    plan = []
    for ros_act in msg.plan:
        name = ros_act.type
        if name == "move_free":
            args = (
                ros_to_body_config(ros_act.q1, robot),
                ros_to_body_config(ros_act.q2, robot),
                ros_to_traj(ros_act, robot, pddl_block_lookup)
            )
        elif name == "move_holding":
            args = (0,0,0,0,
                ros_to_traj(ros_act, robot, pddl_block_lookup))
        elif name == "pick":
            args = (0,0,0,0,0,
                ros_to_traj(ros_act, robot, pddl_block_lookup))
        elif name == "place":
            args = (0,0,0,0,0,0,0,
                ros_to_traj(ros_act, robot, pddl_block_lookup))
        # TODO: Note we are padding the action tuples with zeros since executing
        # each action only really requires the final argument. In future, we 
        # should be more thorough about passing back *all* the information.

        act = (name, args)
        plan.append(act)
        print(act)
    
    return plan
