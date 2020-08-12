#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

/*
Converts a float64 to a SyncWriteItem message that controlls the position of the gripper

Postition:
                Completely Open   Completely Closed
Float64         0                 1.0
Uint Position   0                 738


Force:  (default 0.5 in /gripper_handler/default_current parameter)
                None              Max
Float64         0                 1.0
Uint Position   0                 700
*/

class GripperHandler
{
  public:
    ros::NodeHandle n;
    ros::Subscriber position_sub;
    ros::Subscriber force_sub;
    ros::Publisher position_pub; 
    ros::Publisher force_pub;

    void init();
    void spin();
    void position_callback(const std_msgs::Float64::ConstPtr& msg);
    void force_callback(const std_msgs::Float64::ConstPtr& msg);
};

void GripperHandler::init()
{
    position_pub = n.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/direct/sync_write_item", 5);
    force_pub = n.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 5);
    
    //enable torque
    robotis_controller_msgs::SyncWriteItem torque_msg;
    torque_msg.item_name = "torque_enable";
    torque_msg.joint_name.push_back("gripper");
    torque_msg.value.push_back(0);
    force_pub.publish(torque_msg);
    
    //set default force from parameter
    double default_force;
    ros::param::get("/gripper_handler/default_force", default_force);
    robotis_controller_msgs::SyncWriteItem default_force_msg;
    default_force_msg.item_name = "goal_current";
    default_force_msg.joint_name.push_back("gripper");
    default_force_msg.value.push_back(default_force * 700);
    force_pub.publish(default_force_msg);

    position_sub = n.subscribe("gripper_position", 100, &GripperHandler::position_callback, this); 
    force_sub = n.subscribe("gripper_force", 100, &GripperHandler::force_callback, this); 
    
}

void GripperHandler::spin()
{
  ros::spin();
}

void GripperHandler::position_callback(const std_msgs::Float64::ConstPtr& msg) 
{
    if(ros::ok()){

      robotis_controller_msgs::SyncWriteItem goal_position_msg;

      goal_position_msg.item_name = "goal_position";
      goal_position_msg.joint_name.push_back("gripper");
      goal_position_msg.value.push_back((*msg).data * 738);

      position_pub.publish(goal_position_msg);
    }
}

void GripperHandler::force_callback(const std_msgs::Float64::ConstPtr& msg) 
{
    if(ros::ok()){

      robotis_controller_msgs::SyncWriteItem force_msg;

      force_msg.item_name = "goal_current";
      force_msg.joint_name.push_back("gripper");
      force_msg.value.push_back((*msg).data * 700);

      force_pub.publish(force_msg);
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





