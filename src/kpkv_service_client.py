#!/usr/bin/env python
import rospy
from robotidy.srv import kpkv_msg

def set_joint_kpkv(joint_name, kp, kv):
    rospy.wait_for_service(joint_name + '_kpkv_service')
    try:
        kpkv_service = rospy.ServiceProxy(joint_name + '_kpkv_service', kpkv_msg)
        resp = kpkv_service(kp, kv)
        return resp.setting_is_done
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('kpkv_service_client')
    
    while not rospy.is_shutdown():
        joint_name = raw_input("\nEnter the name of the joint (joint1, joint2..., x to quit): ")
        if joint_name == 'x':
            break

        try:
            kp = float(raw_input("Enter the value of kp: "))
            kv = float(raw_input("Enter the value of kv: "))
        except ValueError:
            rospy.logerr("Invalid input for kp or kv. Please enter a number.")
            continue

        setting_done = set_joint_kpkv(joint_name, kp, kv)
        if setting_done:
            rospy.loginfo(f"{joint_name} setting is done.")
        else:
            rospy.loginfo(f"{joint_name} setting is not done.")

if __name__ == '__main__':
    main()
