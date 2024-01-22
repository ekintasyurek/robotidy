#!/usr/bin/env python
import rospy
import actionlib
from robotidy.msg import trajAction, trajResult
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryActionServer:
    def __init__(self):
        self.nh = rospy.NodeHandle()
        self.action_server = actionlib.SimpleActionServer("trajectory_action", trajAction, self.execute_cb, False)
        self.action_server.start()
        self.pos_cmd_publishers = []
        rospy.loginfo("Trajectory Action Server started.")

    def send_joint_commands(self, cmd_jnts):
        for i, cmd in enumerate(cmd_jnts):
            cmd_msg = Float64()
            cmd_msg.data = cmd
            if i < len(self.pos_cmd_publishers):
                self.pos_cmd_publishers[i].publish(cmd_msg)
            else:
                rospy.logwarn("Mismatch in number of joint commanders and publishers.")

    def execute_cb(self, goal):
        rospy.loginfo("Executing Trajectory Action Server callback.")
        trajectory = goal.trajectory
        njnts = len(trajectory.joint_names)

        # Initialize command publishers for each joint
        self.pos_cmd_publishers = [self.nh.advertise('/' + joint_name + '_pos_cmd', Float64, queue_size=1) for joint_name in trajectory.joint_names]

        # Interpolation parameters
        dt = 0.005
        rate_timer = rospy.Rate(1/dt)

        for point in trajectory.points:
            end_time = point.time_from_start.to_sec()
            rospy.sleep(end_time)  # Simulating waiting for the time to pass for this trajectory point
            self.send_joint_commands(point.positions)

        rospy.loginfo("Trajectory execution completed.")
        self.action_server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('trajectory_action_server_node')
    server = TrajectoryActionServer()
    rospy.spin()
