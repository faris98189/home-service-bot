#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(4.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Request robot to move to Pickup location
  goal.target_pose.pose.position.x = 5.5;
  goal.target_pose.pose.position.y = 12;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Robot is going to the take the virtual object");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {

    ROS_INFO("Robot take's  the virtual object from its place");
    // Wait for 5 seconds
    ros::Duration(4).sleep();
	

    // Request robot to move to Dropoff location
    goal.target_pose.pose.position.x = -8.0;
    goal.target_pose.pose.position.y = 4.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Robot is going to deliever the virtual object");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // Robot reached dropoff zone
      ROS_INFO("Robot deliever the virtual object");
    }
    else
    {
      ROS_INFO("Unable to get to deliever the virtual object");
    }
  }
  else
  {
    ROS_INFO("Unable to talk  the virtual object");
  }

  return 0;
}