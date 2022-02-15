#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Basic Modules
import os
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from robot_vision_msgs.msg import BoundingBoxes
from turtlebot_msgs.srv import SetFollowState

obj = ""
target = ""
x_min = 0
x_max = 0

class MissionDemo(object):

    def __init__(self):
        # Flags
        self._FLAG_EXECUTE = None
        self._FLAG_FOUND = None
        self._FLAG_NAVI = None
        # Soundplay parameter
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.speaker = SoundClient(blocking=True)
        rospy.sleep(1)
        rospy.init_node("mission_demo", disable_signals=True)

    def main_loop(self):
        # Initial
        self.arm_cmd_pub = rospy.Publisher("arm_cmd", String, queue_size=1)
        self.base_cmd_pub = rospy.Publisher("base_cmd", String, queue_size=1)
        rospy.Subscriber("nav_cmd", String, self._navi_callback)
        rospy.Subscriber("lm_data", String, self._voice_callback)
        rospy.Subscriber("yolo_ros/bounding_boxes", BoundingBoxes, self._vision_callback)
        rospy.sleep(1)
        # Set arm to rest-state
        arm_cmd = "rest"
        self.arm_cmd_pub.publish(arm_cmd)
        rospy.sleep(3)
        rospy.loginfo("Waiting for command")
        self.speaker.say("I'm ready for commands", self.voice)
        # Navigating to finding-point
        while (True):
            if self._FLAG_NAVI == 1:
                os.system("gnome-terminal -- bash -c 'rosrun rchomeedu_navigation nav1.py'")
                rospy.sleep(2)
                self.speaker.say("Going to the dining table", self.voice)
                break
        # Reached the point then shut naving
        while (True):
            if self._FLAG_NAVI == 0:
                os.system("rosnode kill /navi_point")
                rospy.sleep(2)
                self.speaker.say("Reached the dining table. Ready to move on", self.voice)
                break
        # Detect the target object
        while (True):
            if self._FLAG_EXECUTE == 1:
                rospy.loginfo("I am seeking the {}".format(target))
                self.speaker.say("I am seeking the {}".format(target), self.voice)
                base_cmd = "whirl"
                self.base_cmd_pub.publish(base_cmd)
                break
        # Seek the target object by yolo
        while (True):
            if self._FLAG_FOUND == 1:
                base_cmd = "stop"
                self.base_cmd_pub.publish(base_cmd)
                rospy.loginfo("I have found the target object {}".format(target))
                self.speaker.say("I have found the target object {}".format(target), self.voice)
                break
        # Approach to the target object
        base_cmd = "forward"
        self.base_cmd_pub.publish(base_cmd)
        rospy.sleep(2)
        base_cmd = "stop"
        self.base_cmd_pub.publish(base_cmd)
        rospy.sleep(1)
        os.system("rosnode kill /chassis_control")
        rospy.sleep(1)
        # Determine the position of the object relative to the robot
        xx = (x_min + x_max)*0.5
        if xx <= 260:
            arm_cmd = "left"
            self.arm_cmd_pub.publish(arm_cmd)
            rospy.sleep(2)
            rospy.loginfo("The {} is on my front left".format(target))
            self.speaker.say("The {} is on my front left".format(target), self.voice)
            rospy.sleep(3)
        elif xx >= 380:
            arm_cmd = "right"
            self.arm_cmd_pub.publish(arm_cmd)
            rospy.sleep(2)
            rospy.loginfo("The {} is on my front right".format(target))
            self.speaker.say("The {} is on my front right".format(target), self.voice)
            rospy.sleep(3)
        else:
            rospy.loginfo("The {} is in front of me".format(target))
            self.speaker.say("The {} is in front of me".format(target), self.voice)
            rospy.sleep(2)
        # Set arm to catch-state to carry 
        arm_cmd = "catch"
        self.arm_cmd_pub.publish(arm_cmd)
        rospy.loginfo("Would you please pass me the {}".format(target))
        self.speaker.say("Would you please pass me the {}".format(target), self.voice)
        # Waiting to go back home
        while (True):
            if self._FLAG_NAVI == 1:
                os.system("gnome-terminal -- bash -c 'rosrun rchomeedu_navigation nav2.py'")
                rospy.sleep(2)
                rospy.loginfo("I'm going to go back to my start-point")
                self.speaker.say("I'm going to go back to my start-point", self.voice)
                break
        # Reached home and that's all
        while (True):
            if self._FLAG_NAVI == 0:
                os.system("rosnode kill /navi_point")
                rospy.sleep(2)
                rospy.loginfo("Reached the start-point")
                rospy.loginfo("I am done here")
                self.speaker.say("Reached the start-point. I am done here", self.voice)
                break
        #rospy.spin()

    def control_follow(self, msg):
        rospy.wait_for_service("/turtlebot_follower/change_state")
        change_state = rospy.ServiceProxy("/turtlebot_follower/change_state", SetFollowState)
        response = change_state(msg)

    def _navi_callback(self, msg):
        if msg.data.find("done") > -1:
            self._FLAG_NAVI = 0

    def _voice_callback(self, msg):
        global target
        if msg.data.find("FIND-BAG") > -1 or msg.data.find("FIND-HANDBAG") > -1:
            target = "handbag"
            self._FLAG_EXECUTE = 1
        elif msg.data.find("FIND-BOTTLE") > -1:
            target = "bottle"
            self._FLAG_EXECUTE = 1
        elif msg.data.find("DINING") > -1 and msg.data.find("TABLE") > -1:
            self._FLAG_NAVI = 1
        elif msg.data.find("HERE-YOU-ARE") > -1:
            self.speaker.say("Thanks. I will take it out for you. Say the command 'follow me' please.")
            os.system("rosnode kill /yolo_ros")
        elif msg.data.find("THANK-YOU") > -1:
            arm_cmd = "rest"
            self.arm_cmd_pub.publish(arm_cmd)
            self.speaker.say("You are welcome. Bye-bye!")
            self._FLAG_NAVI = 1
        elif msg.data.find("FOLLOW-ME") > -1:
            self.speaker.say("OK. I will follow you.")
            self.control_follow(1)
        elif msg.data.find("STOP-FOLLOW") > -1:
            self.speaker.say("Here I will stop following you.")
            self.control_follow(0)
        else:
            self._FLAG_EXECUTE = 0

    def _vision_callback(self, msg):
        global obj, x_min, x_max
        for box in msg.bounding_boxes:
            obj = box.Class
            if obj == target:
                x_min = box.xmin
                x_max = box.xmax
                self._FLAG_FOUND = 1
            else:
                self._FLAG_FOUND = 0

if __name__ == "__main__":
    controller = MissionDemo()
    controller.main_loop()
