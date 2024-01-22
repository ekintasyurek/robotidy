#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties
from sensor_msgs.msg import JointState
from robotidy.srv import kpkv_msg, kpkv_msgRequest, kpkv_msgResponse

class Joint:
    def __init__(self, joint_name, dt):
        self.joint_name = joint_name
        self.pos_cmd = 0.0
        self.duration = rospy.Duration(dt)
        self.kp = 10.0
        self.kv = 3.0

        # Initialize ROS services and publishers
        self.get_jnt_state_client = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.set_trq_client = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

        self.pos_publisher = rospy.Publisher(joint_name + "_pos", Float64, queue_size=1)
        self.vel_publisher = rospy.Publisher(joint_name + "_vel", Float64, queue_size=1)
        self.trq_publisher = rospy.Publisher(joint_name + "_trq", Float64, queue_size=1)
        self.joint_state_publisher = rospy.Publisher(joint_name + "_states", JointState, queue_size=1)

        self.pos_cmd_subscriber = rospy.Subscriber(joint_name + "_pos_cmd", Float64, self.pos_cmd_callback)
        self.kpkv_server = rospy.Service(joint_name + "_kpkv_service", kpkv_msg, self.kpkv_callback)

    def pos_cmd_callback(self, msg):
        self.pos_cmd = msg.data

    def get_joint_state(self):
        resp = self.get_jnt_state_client(self.joint_name)
        self.pos_cur = resp.position[0]
        self.vel_cur = resp.rate[0]

        # Publish joint state
        pos_msg = Float64()
        pos_msg.data = self.pos_cur
        self.pos_publisher.publish(pos_msg)

        vel_msg = Float64()
        vel_msg.data = self.vel_cur
        self.vel_publisher.publish(vel_msg)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name.append(self.joint_name)
        joint_state_msg.position.append(self.pos_cur)
        joint_state_msg.velocity.append(self.vel_cur)
        self.joint_state_publisher.publish(joint_state_msg)

    def joint_trq_control(self):
        pos_err = self.pos_cmd - self.pos_cur
        # Watch for periodicity
        if pos_err > math.pi:
            pos_err -= 2 * math.pi
        if pos_err < -math.pi:
            pos_err += 2 * math.pi

        self.trq_cmd = self.kp * pos_err - self.kv * self.vel_cur

        # Publish torque
        trq_msg = Float64()
        trq_msg.data = self.trq_cmd
        self.trq_publisher.publish(trq_msg)

        # Send torque command to Gazebo
        try:
            self.set_trq_client(self.joint_name, self.trq_cmd, self.duration)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call to apply_joint_effort failed: %s" % e)

    def kpkv_setting(self, kp, kv):
        self.kp = kp
        self.kv = kv

    def kpkv_callback(self, request):
        self.kpkv_setting(request.kp, request.kv)
        rospy.loginfo("%s: kp set to %f, kv set to %f" % (self.joint_name, self.kp, self.kv))
        return kpkv_msgResponse(setting_is_done=True)


if __name__ == '__main__':
    rospy.init_node('two_joints_controller')
    nh = rospy.Rate(100)  # 100 Hz
    dt = 0.01  # sample time for the controller

    # Instantiate joint instances
    joint1 = Joint("joint1", dt)
    joint2 = Joint("joint2", dt)
    joint3 = Joint("joint3", dt)
    joint4 = Joint("joint4", dt)
    joint5 = Joint("joint5", dt)
    joint6 = Joint("joint6", dt)

    # Example kpkv settings
    joint1.kpkv_setting(50, 15)
    joint2.kpkv_setting(50, 15)
    joint3.kpkv_setting(30, 9)
    joint4.kpkv_setting(30, 9)
    joint5.kpkv_setting(30, 9)
    joint6.kpkv_setting(30, 9)

    while not rospy.is_shutdown():
        joint1.get_joint_state()
        joint2.get_joint_state()
        joint3.get_joint_state()
        joint4.get_joint_state()
        joint5.get_joint_state()
        joint6.get_joint_state()

        joint1.joint_trq_control()
        joint2.joint_trq_control()
        joint3.joint_trq_control()
        joint4.joint_trq_control()
        joint5.joint_trq_control()
        joint6.joint_trq_control()

        rospy.spinOnce()
        nh.sleep()
