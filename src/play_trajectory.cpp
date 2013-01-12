#include <trajectory_recorder/arm.h>

int main(int argc, char**argv)
{
  ros::init(argc,argv,"arm");
  ros::Subscriber sub;
  Arm rarm("right", sub);
  std::vector<std::vector<double> > path;
  double dur = 0.3;
  std::vector<double> waypoint_durations;

  ROS_INFO("Parsing right arm text file.");
  if(!rarm.parseTrajectoryFile("/home/wmarshall/workspace/trajectory_recorder/motions/place_motion.txt", path))
    ROS_ERROR("Failed to parse trajectory.");
  else
  {
    ROS_INFO("Trajectory has length: %d", int(path.size()));
    rarm.printTrajectory("right_arm_trajectory", path);
    ROS_INFO("Going to first waypoint...");
    double first[7];
    for(size_t j = 0; j < 7; ++j)
      first[j] = path[0][j];

    rarm.sendArmToConfiguration(first, 1.0);
    sleep(2);
    waypoint_durations.push_back(dur);
    for(size_t i = 1; i < path.size(); ++i)
      waypoint_durations.push_back(double(i)*dur + dur);
    //rarm.sendArmToConfigurations(path, waypoint_durations);
    
    trajectory_msgs::JointTrajectory traj, filtered_traj;
    rarm.pathToJointTrajectory(path, waypoint_durations, traj);
    ROS_INFO("Filtering...");
    /*if(!rarm.filterTrajectory(traj, filtered_traj))
    {
      ROS_ERROR("Failed to filter trajectory. Exiting.");
      return false;
      }*/
    ROS_INFO("Executing...");
    rarm.sendArmToJointTrajectory(traj);
  }

  sleep(2);
  return 0;
}


