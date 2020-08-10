#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

/*
Converts a float64 to a SyncWriteItem message that controlls the position of the gripper

                Completely Open   Completely Closed
Float64         0                 1.0
Uint Position   0                 738
*/

class GripperHandler
{
  public:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    void init();
    void spin();
    void callback(const std_msgs::Float64::ConstPtr& msg);
};

void GripperHandler::init()
{
    pub = n.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/direct/sync_write_item", 100);
    sub = n.subscribe("gripper_position", 100, &GripperHandler::callback, this); 
}

void GripperHandler::spin()
{
  ros::spin();
}

void GripperHandler::callback(const std_msgs::Float64::ConstPtr& msg) 
{
    if(ros::ok()){

      robotis_controller_msgs::SyncWriteItem goal_position_msg;

      goal_position_msg.item_name = "goal_position";
      goal_position_msg.joint_name.push_back("gripper");
      goal_position_msg.value.push_back((*msg).data * 738);

      pub.publish(goal_position_msg);
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper_handler");

  GripperHandler handler;

  handler.init();
  handler.spin();

  return 0;
}





