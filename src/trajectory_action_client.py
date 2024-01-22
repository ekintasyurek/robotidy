#!/usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotidy.msg import trajAction, trajGoal
from gazebo_msgs.srv import GetJointProperties, GetModelState


def inverse_kinematics(gripper_pos, gripper_dist):
    # Define robot parameters
    link1_height = 1.1
    link2_3_length = 1.0
    gripper_center = 0.15
    gripper_radius = 0.04
    gripper_range = 0.16

    cyl_r = math.sqrt(gripper_pos.x**2 + gripper_pos.y**2)
    cyl_theta = math.atan2(gripper_pos.y, gripper_pos.x)
    cyl_z = gripper_pos.z

    theta1 = math.atan((cyl_z - link1_height) / (cyl_r - gripper_center))
    theta2 = math.acos(math.sqrt((cyl_z - link1_height)**2 + (cyl_r - gripper_center)**2) / 2)
    half_dist = gripper_dist / 2 + gripper_radius

    cmd_jnts = [0] * 6
    cmd_jnts[0] = cyl_theta
    cmd_jnts[1] = math.pi/2 - (theta1 + theta2)
    cmd_jnts[2] = 2 * theta2
    cmd_jnts[3] = math.pi/2 - cmd_jnts[1] - cmd_jnts[2]
    cmd_jnts[4] = -(gripper_range - half_dist)
    cmd_jnts[5] = (gripper_range - half_dist)

    return cmd_jnts


def prepare_trajectory(start_jnts, end_jnts, time_steps):
    trajectory = JointTrajectory()
    trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    points = []
    for i in range(time_steps + 1):
        point = JointTrajectoryPoint()
        fraction = float(i) / time_steps
        point.positions = [start_jnts[j] + (end_jnts[j] - start_jnts[j]) * fraction for j in range(6)]
        point.time_from_start = rospy.Duration(i)
        points.append(point)

    trajectory.points = points
    return trajectory


def move_robot(action_client, trajectory, wait_time):
    goal = trajGoal()
    goal.trajectory = trajectory
    action_client.send_goal(goal)
    return action_client.wait_for_result(rospy.Duration(wait_time))


if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_action_client_node')

        # Initialize action client
        client = actionlib.SimpleActionClient('trajectory_action', trajAction)
        client.wait_for_server()

        get_jnt_state_client = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        get_model_state_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Define task parameters
        time_delay = 1.0
        lift_height = 0.3
        gripper_open = 0.24
        gripper_close = 0.08
        table_height = 1.0
        beer_height = 0.28

        # Fetch initial joint positions
        origin_jnts = [get_jnt_state_client(joint_name).position[0] for joint_name in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]]

        # Define safe position
        safe_jnts = [0, 30.0/180.0*math.pi, 75.0/180.0*math.pi, math.pi/2 - 105.0/180.0*math.pi, 0, 0]

        # Task 1: Move to safe point
        rospy.loginfo("Task 1: Moving to safe point.")
        trajectory = prepare_trajectory(origin_jnts, safe_jnts, 5)
        move_robot(client, trajectory, 7)
        rospy.sleep(time_delay)

        # Task 2: Move to above beer
        rospy.loginfo("Task 2: Moving to above beer.")
        beer_pos = get_model_state_client(model_name="beer", relative_entity_name="link").pose.position
        hover_open_start_pos = Point(beer_pos.x, beer_pos.y, beer_pos.z + beer_height/2 + lift_height)
        hover_open_start_jnts = inverse_kinematics(hover_open_start_pos, gripper_open)
        trajectory = prepare_trajectory(safe_jnts, hover_open_start_jnts, 5)
        move_robot(client, trajectory, 7)
        rospy.sleep(time_delay)

        # Task 3: Move the gripper around the beer
        rospy.loginfo("Task 3: Moving the gripper around the beer.")
        around_open_start_pos = Point(beer_pos.x, beer_pos.y, beer_pos.z + beer_height / 2)
        around_open_start_jnts = inverse_kinematics(around_open_start_pos, gripper_open)
        start_jnts = hover_open_start_jnts
        end_jnts = around_open_start_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 3)
        move_robot(client, trajectory, 5)
        rospy.sleep(time_delay)

        # Task 4: Clamp the gripper to grasp the beer
        rospy.loginfo("Task 4: Clamping the gripper to grasp the beer.")
        grasp_close_start_pos = around_open_start_pos
        grasp_close_start_jnts = inverse_kinematics(grasp_close_start_pos, gripper_close)
        start_jnts = around_open_start_jnts
        end_jnts = grasp_close_start_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 2)
        move_robot(client, trajectory, 4)
        rospy.sleep(time_delay)

        # Task 5: Move the gripper up with the beer
        rospy.loginfo("Task 5: Moving the gripper up with the beer.")
        hover_close_start_pos = Point(grasp_close_start_pos.x, grasp_close_start_pos.y, grasp_close_start_pos.z + lift_height)
        hover_close_start_jnts = inverse_kinematics(hover_close_start_pos, gripper_close)
        start_jnts = grasp_close_start_jnts
        end_jnts = hover_close_start_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 3)
        move_robot(client, trajectory, 5)
        rospy.sleep(time_delay)

        # Task 6: Move the gripper to the above of target area
        rospy.loginfo("Task 6: Moving the gripper to the above of the target area.")
        table_pos = get_model_state_client(model_name="table", relative_entity_name="link").pose.position
        target_pos = Point(table_pos.x, table_pos.y, table_pos.z + table_height)
        hover_close_end_pos = Point(target_pos.x, target_pos.y, target_pos.z + beer_height/2 + lift_height)
        hover_close_end_jnts = inverse_kinematics(hover_close_end_pos, gripper_close)
        start_jnts = hover_close_start_jnts
        end_jnts = hover_close_end_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 5)
        move_robot(client, trajectory, 7)
        rospy.sleep(time_delay)

        # Task 7: Move the gripper down to place the beer on the table
        rospy.loginfo("Task 7: Moving the gripper down to place the beer on the table.")
        grasp_close_end_pos = Point(target_pos.x, target_pos.y, target_pos.z + beer_height/2)
        grasp_close_end_jnts = inverse_kinematics(grasp_close_end_pos, gripper_close)
        start_jnts = hover_close_end_jnts
        end_jnts = grasp_close_end_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 3)
        move_robot(client, trajectory, 5)
        rospy.sleep(time_delay)

        # Task 8: Unclamp the gripper and release the beer
        rospy.loginfo("Task 8: Unclamping the gripper to release the beer.")
        around_open_end_pos = Point(grasp_close_end_pos.x, grasp_close_end_pos.y, grasp_close_end_pos.z)
        around_open_end_jnts = inverse_kinematics(around_open_end_pos, gripper_open)
        start_jnts = grasp_close_end_jnts
        end_jnts = around_open_end_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 2)
        move_robot(client, trajectory, 4)
        rospy.sleep(time_delay)

        # Task 9: Move the gripper up from the table
        rospy.loginfo("Task 9: Moving the gripper up from the table.")
        hover_open_end_pos = Point(around_open_end_pos.x, around_open_end_pos.y, around_open_end_pos.z + lift_height)
        hover_open_end_jnts = inverse_kinematics(hover_open_end_pos, gripper_open)
        start_jnts = around_open_end_jnts
        end_jnts = hover_open_end_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 3)
        move_robot(client, trajectory, 5)
        rospy.sleep(time_delay)

        # Task 10: Move the gripper back to the safe point
        rospy.loginfo("Task 10: Moving the gripper back to the safe point.")
        start_jnts = hover_open_end_jnts
        end_jnts = safe_jnts
        trajectory = prepare_trajectory(start_jnts, end_jnts, 5)
        move_robot(client, trajectory, 7)
        rospy.sleep(time_delay)

        rospy.loginfo("Trajectory action completed.")

    except rospy.ROSInterruptException:
        pass
