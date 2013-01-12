#include <trajectory_recorder/arm.h>

int main(int argc, char**argv)
{
  
  ros::init(argc,argv,"arm");
  ros::Subscriber sub;
  Arm rarm("right", sub);
  
  
  //rarm.sub_ = rarm.nh_.subscribe("/r_arm_controller/state", 10, &Arm::armControllerCallback, &rarm);
  std::vector<std::vector<double> > path;
  double dur = 0.3;
  std::vector<double> waypoint_durations;
  std::vector<double> move_times;
  ROS_INFO("Parsing right arm text file.");
  if(!rarm.parseTrajectoryFile("/home/wmarshall/workspace/trajectory_recorder/motions/default.txt", path))
    ROS_ERROR("Failed to parse trajectory.");
  else
  {
    ROS_INFO("Trajectory has length: %d", int(path.size()));
    rarm.printTrajectory("right_arm_trajectory", path);
    ROS_INFO("Going to first waypoint...");
    double first[7];
    double primitive=0.13962634016;
    for(size_t j = 0; j < 7; ++j)
      first[j] = path[0][j];
    double d[]={0.0,0.0,0.0,-0.3,0.0,0.0,0.0};
    /*for(int double delta0 = -2.135, delta1=(-2.135+primitive); delta1<0.564; delta0=delta1, delta1+=primitive){
      ROS_INFO("%f", delta1);
      d[0]=delta0; first[0]=delta1;
      move_times.push_back(rarm.time(d, first, 2.0));
      }*/
    ROS_INFO("The motion took %f seconds", rarm.time(d, first, 0.0));
    //sleep(2);
    first[0]=.5;
    ROS_INFO("The motion took %f seconds", rarm.time(d, first, 0.0));
    //waypoint_durations.push_back(dur);
    //for(size_t i = 1; i < path.size(); ++i)
    //  waypoint_durations.push_back(double(i)*dur + dur);
    //rarm.sendArmToConfigurations(path, waypoint_durations);
    
    //trajectory_msgs::JointTrajectory traj, filtered_traj;
    //rarm.pathToJointTrajectory(path, waypoint_durations, traj);
    /*ROS_INFO("Filtering...");
    if(!rarm.filterTrajectory(traj, filtered_traj))
    {
      ROS_ERROR("Failed to filter trajectory. Exiting.");
      return false;
      }*/
    //ROS_INFO("Executing...");
    //rarm.sendArmToJointTrajectory(traj);
  }

  ros::shutdown();
  return 0;
}


