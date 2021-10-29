#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "/home/mustar/catkin_ws/devel/include/dynamixel_msgs/JointState.h"

class ArmControl
{
public:
  ArmControl();

private:
  void cmdCallback(const std_msgs::String::ConstPtr &cmd);
  void waistCallback(const dynamixel_msgs::JointState::ConstPtr &waiststate);
  void shoulderCallback(const dynamixel_msgs::JointState::ConstPtr &shoulderstate);
  void elbowCallback(const dynamixel_msgs::JointState::ConstPtr &elbowstate);
  void wristCallback(const dynamixel_msgs::JointState::ConstPtr &wriststate);
  void handCallback(const dynamixel_msgs::JointState::ConstPtr &handstate);
  void publish();

  ros::NodeHandle ph_, nh_;

  ros::Publisher waist_pub_;
  ros::Publisher shoulder_pub_;
  ros::Publisher elbow_pub_;
  ros::Publisher wrist_pub_;
  ros::Publisher hand_pub_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber waist_sub_;
  ros::Subscriber shoulder_sub_;
  ros::Subscriber elbow_sub_;
  ros::Subscriber wrist_sub_;
  ros::Subscriber hand_sub_;

  std_msgs::Float64 waist_published_;
  std_msgs::Float64 waist;
  std_msgs::Float64 shoulder_published_;
  std_msgs::Float64 shoulder;
  std_msgs::Float64 elbow_published_;
  std_msgs::Float64 elbow;
  std_msgs::Float64 wrist_published_;
  std_msgs::Float64 wrist;
  std_msgs::Float64 hand_published_;
  std_msgs::Float64 hand;

  ros::Timer timer_;
};

ArmControl::ArmControl()
{
  waist_pub_ = ph_.advertise<std_msgs::Float64>("waist_controller/command", 1, true);
  shoulder_pub_ = ph_.advertise<std_msgs::Float64>("shoulder_controller/command", 1, true);
  elbow_pub_ = ph_.advertise<std_msgs::Float64>("elbow_controller/command", 1, true);
  wrist_pub_ = ph_.advertise<std_msgs::Float64>("wrist_controller/command", 1, true);
  hand_pub_ = ph_.advertise<std_msgs::Float64>("hand_controller/command", 1, true);

  waist_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("waist_controller/state", 10, &ArmControl::waistCallback, this);
  shoulder_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("shoulder_controller/state", 10, &ArmControl::shoulderCallback, this);
  elbow_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("elbow_controller/state", 10, &ArmControl::elbowCallback, this);
  wrist_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("wrist_controller/state", 10, &ArmControl::wristCallback, this);
  hand_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("hand_controller/state", 10, &ArmControl::handCallback, this);

  cmd_sub_ = nh_.subscribe<std_msgs::String>("arm_cmd", 10, &ArmControl::cmdCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ArmControl::publish, this));
}

void ArmControl::waistCallback(const dynamixel_msgs::JointState::ConstPtr &waiststate)
{
  waist.data = waiststate->goal_pos;
}

void ArmControl::shoulderCallback(const dynamixel_msgs::JointState::ConstPtr &shoulderstate)
{
  shoulder.data = shoulderstate->goal_pos;
}

void ArmControl::elbowCallback(const dynamixel_msgs::JointState::ConstPtr &elbowstate)
{
  elbow.data = elbowstate->goal_pos;
}

void ArmControl::wristCallback(const dynamixel_msgs::JointState::ConstPtr &wriststate)
{
  wrist.data = wriststate->goal_pos;
}

void ArmControl::handCallback(const dynamixel_msgs::JointState::ConstPtr &handstate)
{
  hand.data = handstate->goal_pos;
}

void ArmControl::cmdCallback(const std_msgs::String::ConstPtr &cmd)
{
  if (cmd->data == "left")
  {
    waist.data = 0.4;
    shoulder.data = 0.5;
    elbow.data = 0.4;
    wrist.data = 0.5;
    hand.data = 0.0;
  }
  if (cmd->data == "right")
  {
    waist.data = -0.4;
    shoulder.data = 0.5;
    elbow.data = 0.4;
    wrist.data = 0.5;
    hand.data = 0.0;
  }
  if (cmd->data == "catch")
  {
    waist.data = 0.0;
    shoulder.data = -0.5;
    elbow.data = 1.2;
    wrist.data = 0.8;
    hand.data = 0.0;
  }
  if (cmd->data == "rest")
  {
    waist.data = 0.0;
    shoulder.data = -1.6;
    elbow.data = 2.0;
    wrist.data = 1.2;
    hand.data = 0.0;
  }
  if (cmd->data == "open")
  {
    hand.data = -0.2;
  }
  if (cmd->data == "close")
  {
    hand.data = 0.2;
  }
  waist_published_ = waist;
  shoulder_published_ = shoulder;
  wrist_published_ = wrist;
  elbow_published_ = elbow;
  hand_published_ = hand;
}

void ArmControl::publish()
{
  waist_pub_.publish(waist_published_);
  shoulder_pub_.publish(shoulder_published_);
  wrist_pub_.publish(wrist_published_);
  elbow_pub_.publish(elbow_published_);
  hand_pub_.publish(hand_published_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_control");
  ArmControl arm_control;
  ros::spin();
}
