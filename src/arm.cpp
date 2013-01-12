#include <trajectory_recorder/arm.h>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <boost/thread.hpp>

Arm::Arm(std::string arm_name, ros::Subscriber sub):sub_(sub)
{
  reference_frame_ = "base_footprint";
  
  arm_name_ = arm_name;

  if(arm_name_.compare("left") == 0)
  {
    group_name_ = "left_arm";
    ik_service_name_ = "/pr2_left_arm_kinematics/get_ik";
    fk_service_name_ = "/pr2_left_arm_kinematics/get_fk";
    controller_state_name_ = "l_arm_controller/state";
    joint_names_.push_back("l_shoulder_pan_joint");
    joint_names_.push_back("l_shoulder_lift_joint");
    joint_names_.push_back("l_upper_arm_roll_joint");
    joint_names_.push_back("l_elbow_flex_joint");
    joint_names_.push_back("l_forearm_roll_joint");
    joint_names_.push_back("l_wrist_flex_joint");
    joint_names_.push_back("l_wrist_roll_joint");
    traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
  }
  else if (arm_name_.compare("right") == 0)
  {
    group_name_ = "right_arm";
    ik_service_name_ = "/pr2_right_arm_kinematics/get_ik";
    fk_service_name_ = "/pr2_right_arm_kinematics/get_fk";
    controller_state_name_ = "r_arm_controller/state";
    joint_names_.push_back("r_shoulder_pan_joint");
    joint_names_.push_back("r_shoulder_lift_joint");
    joint_names_.push_back("r_upper_arm_roll_joint");
    joint_names_.push_back("r_elbow_flex_joint");
    joint_names_.push_back("r_forearm_roll_joint");
    joint_names_.push_back("r_wrist_flex_joint");
    joint_names_.push_back("r_wrist_roll_joint");
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
  }
  else
  {
  	ROS_ERROR("Arm Class was initialized with %s, needs to be right or left", arm_name_.c_str());
  }

  while(!traj_client_->waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the joint_trajectory_action server");


  /*Set the max and min angles */
  max = (double *)malloc(7*sizeof(double));
  min = (double *)malloc(7*sizeof(double));
		      
  min[0]= -2.135; max[0] = 0.564; 
  min[1]= -0.353; max[1] = 1.170;
  min[2]= -3.750; max[2] = 0.645;
  min[3]= -2.121; max[3] =-0.150;
  min[4]= -3.141; max[4] = 3.141;
  min[5]= -1.999; max[5] =-0.100;
  min[6]= -3.141; max[6] = 3.141;
  useError = (bool *)malloc(sizeof(bool)*7);
  current_goal = (double *)malloc(7*sizeof(double));
  for(int i = 0; i < 7;i++){
    current_goal[i]=0.0;
    useError[i]=false;
  }
  withinError=false;
  started=false;
  sub_ = nh_.subscribe("/r_arm_controller/state", 10, &Arm::armControllerCallback, this);
  //spinner = ros::MultiThreadedSpinner(0);
  //tid= boost::thread(boost::bind( &Arm::subscriberThread, this));
  aspinner= new ros::AsyncSpinner(0);
  aspinner->start();
  mux.lock();
  pos_thresh=0.05;
  vel_thresh=10.09;
  
  ROS_INFO("Initialized successfully initialized.");
}

Arm::~Arm()
{
  if(mux.try_lock()){
    mux.unlock();
  }
  else
    mux.unlock();
  if(spin_once.try_lock()){
    spin_once.unlock();
  }
  else
    spin_once.unlock();
  delete traj_client_;
}

/*void Arm::subscriberThread(){
  spinner.spin();
  }*/

void Arm::stopArm()
{
	ROS_INFO("Stopping Arm by canceling all trajectory goals.");
	traj_client_->cancelAllGoals();
}

void Arm::sendArmToConfiguration(double configuration[7], double move_time)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);
 
  for(unsigned int i = 0; i < 7; i++)
  {
    goal.trajectory.points[0].positions[i] = configuration[i];
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  goal.trajectory.points[0].velocities.resize(7);
//  for (size_t j = 0; j < 7; ++j)
//    goal.trajectory.points[0].velocities[j] = 1;

  goal.trajectory.points[0].time_from_start = ros::Duration(move_time);
  //ROS_INFO("Beginning to time...");
  //double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  //traj_client_->waitForResult(ros::Duration(move_time*4));
  //ROS_INFO("It took the controller %f seconds to execute the first goal(no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::cohereAngles(double angles[7]){
  double two_pi = 2*3.14159265897;
  double pi= 3.14159265897;
  for(int i = 0; i < 7;i++){
    while(angles[i]<-1*pi||angles[i]>pi){      
      if(angles[i]<-1*pi){
	angles[i]+=two_pi;
      }
      else if(angles[i]>=pi){
	angles[i]-=two_pi;
      }
    }
  }
}

void Arm::cohereAngles(vector<double> &angles){
  double two_pi = 2*3.14159265897;
  double pi= 3.14159265897;
  for(int i = 0; i < 7;i++){
    while(angles[i]<-1*pi||angles[i]>pi){      
      if(angles[i]<-1*pi){
	angles[i]+=two_pi;
      }
      else if(angles[i]>=pi){
	angles[i]-=two_pi;
      }
    }
  }
}

void Arm::makePr2Angles(double angles[7]){
  for(int i = 0; i < 7; i++){
    if(angles[i]>max[i]){
      angles[i] = max[i];   
    }
    if(angles[i]<min[i]){
      angles[i] = min[i];
    }
  }
}

void Arm::makePr2Angles(vector<double> &angles){
  for(int i = 0; i < 7; i++){
    if(angles[i]>max[i]){
      angles[i] = max[i];   
    }
    if(angles[i]<min[i]){
      angles[i] = min[i];
    }
  }
}

double Arm::time(double initial[], double final[], double move_time){
  pr2_controllers_msgs::JointTrajectoryGoal initial_goal;
  pr2_controllers_msgs::JointTrajectoryGoal final_goal;
  pos_thresh=0.03;
  vel_thresh=0.3;
  static bool firstcall=true;
  cohereAngles(initial);
  //printArray("final",final);
  cohereAngles(final);
  //printArray("final", final);
  makePr2Angles(initial);
  //printArray("final", final);
  makePr2Angles(final);
  //printArray("final", final);
  initial_goal.trajectory.header.seq = 0;
  initial_goal.trajectory.header.stamp = ros::Time::now();
  initial_goal.trajectory.header.frame_id = reference_frame_;

  initial_goal.trajectory.points.resize(1);
  initial_goal.trajectory.points[0].positions.resize(7);
 
  for(unsigned int i = 0; i < 7; i++)
  {
    initial_goal.trajectory.points[0].positions[i] = initial[i];
    initial_goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  initial_goal.trajectory.points[0].velocities.resize(7);
  initial_goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

  final_goal.trajectory.header.seq = 0;
  final_goal.trajectory.header.stamp = ros::Time::now();
  final_goal.trajectory.header.frame_id = reference_frame_;

  final_goal.trajectory.points.resize(1);
  final_goal.trajectory.points[0].positions.resize(7);
 
  for(unsigned int i = 0; i < 7; i++)
  {
    final_goal.trajectory.points[0].positions[i] = final[i];
    final_goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  final_goal.trajectory.points[0].velocities.resize(7);
  final_goal.trajectory.points[0].time_from_start = ros::Duration(move_time);
  setCurrentGoal(initial, true);
  traj_client_->sendGoal(initial_goal);
  started=true;
  while(spin_once.try_lock()){
    spin_once.unlock();
  }
  spin_once.lock();
  spin_once.unlock();
  mux.lock();
  mux.unlock();
  
  while(ros::Time::now().toSec()==0.0);
  if(firstcall){
    sleep(1);
    firstcall=false;
  }
  
  setCurrentGoal(final,true);
  withinError=false;
  traj_client_->sendGoal(final_goal);
  start = ros::Time::now().toSec();
  
  while(spin_once.try_lock()){
    spin_once.unlock();
  }
  spin_once.lock();
  spin_once.unlock();
  mux.lock();
  mux.unlock();
  
  /*int x=0;
  withinError?x=1:x=0;
  ROS_INFO("Within Error is %d", x);*/
  // ROS_INFO("1");
  //while(!withinError);
  //spinUntilWithinError(final);
  
  withinError=false;
  //ROS_INFO("2");
  //started=false;
  //sleep(3);
  return ros::Time::now().toSec()-start;
}

void Arm::setCurrentGoal(double goal[7], bool usePrecision){
  if(usePrecision){
    for(int i = 0; i < 7; i++){
      double delta = goal[i]-current_goal[i];
      if(delta>0.01)
	useError[i]=true;
      current_goal[i] = goal[i];
    }
  }
  else
    {
      for(int i = 0; i < 7; i++){
	current_goal[i] = goal[i];
	useError[i]=true;
      }
    }
}

double Arm::timePath(vector<vector<double> > points, vector<double> times){
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  //velocity_threshold=10.0;
  pos_thresh=0.24;
  vector<double> recorded_times;
  while(ros::Time::now().toSec()==0.0);
  double start;
  trajectory_msgs::JointTrajectory traj;
  for(int i = 0; i < points.size();i++){
    cohereAngles(points[i]);
    makePr2Angles(points[i]);
  }
  for(int i = 0; i < 7;i++)
    ROS_INFO("%f ", points[1][i]);
  pathToJointTrajectory(points, times, traj);
  goal.trajectory=traj;
  traj_client_->sendGoal(goal);
  start = ros::Time::now().toSec();
  started=true;
  for(int i =0;i < points.size();i++){
    smallest=100.0;
    setCurrentGoal(points[i].data(), false);
    
    while(spin_once.try_lock()){
      spin_once.unlock();
    }
    spin_once.lock();
    spin_once.unlock();
    cout<<"about to block "<<i<<endl;
    mux.lock();
    mux.unlock();
    recorded_times.push_back(ros::Time::now().toSec()-start);
  }
  ROS_INFO("Path times ");
  for(int i = 0; i < times.size(); i++)
    ROS_INFO("%f ", times[i]);  
}

void Arm::printArray(char * s,const double a[7]){
    ROS_INFO("%s: [%f, %f, %f, %f, %f, %f, %f]", s, a[0], a[1], a[2], a[3], a[4], a[5], a[6]); 
}

void Arm::armControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& message){
  spin_once.lock();
  /*static bool first=true;
  if(first){
    ROS_INFO("The start time is %f", ros::Time::now().toSec()-start);
    first=false;
    }*/
  /*if(started){
    ros::Time t = ros::Time::now();
    //ROS_INFO("The start time is %f %f.%f", t.toSec()-start, double(t.sec),double(t.nsec));
    }*/
  static double count = -1.0;
  static double start = ros::Time::now().toSec();
  count+=1.0;
  if(count==500){
    double newtime=ros::Time::now().toSec();
    ROS_INFO("Hz=%f", count/(newtime-start));
    start=newtime;
    count=0.0;
}

  double data[7];
  for(int i = 0; i < 7;i++){
    data[i]=message->actual.positions[i];
  }
  cohereAngles(data);
  makePr2Angles(data);
  float sum=0.0;
  for(int i = 0; i < 7; i++){
    //sum+=(current_goal[i]-message->actual.positions[i])*(current_goal[i]-message->actual.positions[i]);
    double error=0.0;
    if(useError[i])
      error = (current_goal[i]-data[i]);
    sum+=error*error;
    //ROS_ERROR("Sum: %f Error: %f Use: %s", sum, error, useError[i]?"true":"false");
  }
  sum=sqrt(sum);
  if(sum<smallest)
    smallest=sum;
  double sumvel=0.0;
  double vel=0.0;
  for(int i = 0; i < 7;i++){
    vel = message->actual.velocities[i];
    sumvel+=vel*vel;
  }
  vel=sqrt(vel);
  
  if(started&&false){
    
    printArray("Error values", message->error.positions.data());
    printArray("Desired values", message->desired.positions.data());
    printArray("Actual values", message->actual.positions.data());
    printArray("Current goal", (const double *)current_goal);
    ROS_INFO("Sum: %f Sumvel: %f", sum, sumvel);
    printArray("Velocities", message->actual.velocities.data());
  }
  
  //S_INFO("");
  if(sum<pos_thresh&&sumvel<vel_thresh){
    mux.unlock();
    withinError=true;
    //ROS_INFO("Within Error is true");
  }
  else{
    mux.try_lock();
    withinError=false;  
  }
  spin_once.unlock();
  //ROS_INFO("Smallest: (sum,smallest) (%f, %f)",sum, smallest);
}

/*
void callback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr &msg){
  ROS_INFO("Got to callback");
  std::cout<<"You got to the callback"<<std::endl;
  std::ofstream file("file");
  file<<"this is some text"<<std::endl;
  }*/

  
void Arm::spinUntilWithinError(double expected[7]){
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr message = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state");
  double sum=0.0;
  double error[7];
  while(1){
    for(int i = 0; i < 7; i++){
      sum+=(expected[i]-message->actual.positions[i])*(expected[i]-message->actual.positions[i]);
    }
    /*for(int i = 0; i < 7; i++){
      sum+=message->error.positions[i]*message->error.positions[i];
      }*/
    ROS_INFO("sum: %f \nerror values: [%f, %f, %f, %f, %f, %f, %f] \nexpected values: [%f, %f, %f, %f, %f, %f, %f] \nactual values: [%f, %f, %f, %f, %f, %f, %f", sum, message->error.positions[0], message->error.positions[1], message->error.positions[2], message->error.positions[3], message->error.positions[4], message->error.positions[5], message->error.positions[6], message->desired.positions[0], message->desired.positions[1], message->desired.positions[2], message->desired.positions[3], message->desired.positions[4], message->desired.positions[5], message->desired.positions[6], message->actual.positions[0], message->actual.positions[1], message->actual.positions[2], message->actual.positions[3], message->actual.positions[4], message->actual.positions[5], message->actual.positions[6]); 
    if(sum<0.015)
      return;
    sum=0.0;
    message = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>("/r_arm_controller/state");
  }
}


void Arm::sendArmToConfiguration(std::vector<float> configuration, double move_time)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);
 
  for(unsigned int i = 0; i < 7; i++)
  {
    goal.trajectory.points[0].positions[i] = configuration[i];
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  }

  goal.trajectory.points[0].velocities.resize(7);
//  for (size_t j = 0; j < 7; ++j)
//    goal.trajectory.points[0].velocities[j] = 1;

  goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG("sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}



void Arm::sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  goal.trajectory.points.resize(configurations.size());
 
  for(unsigned int i = 0; i < 7; i++)
    goal.trajectory.joint_names.push_back(joint_names_[i]);
  
  for(unsigned int i = 0; i < configurations.size(); i++)
  {
    goal.trajectory.points[i].time_from_start = ros::Duration(move_times[i]);

    goal.trajectory.points[i].positions.resize(7);
    for(unsigned int j = 0; j < 7; j++)
      goal.trajectory.points[i].positions[j] = configurations[i][j];

    // debug output
    printf("% 02d: ", int(i));
    for(unsigned int j = 0; j < 7; j++)
      printf("%0.4f ", configurations[i][j]);
    printf(" time: %0.3f\n", move_times[i]);

    goal.trajectory.points[i].velocities.resize(7,0);
    //for (size_t j = 0; j < 7; ++j)
    //  goal.trajectory.points[i].velocities[j] = 0.0000000; //0.00000001
  }
  
  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_DEBUG("sending goal to controller took %f seconds.", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToJointTrajectory(trajectory_msgs::JointTrajectory &traj)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory = traj;

  // overwrite the header...remove this
  goal.trajectory.header.seq = 0;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.header.frame_id = reference_frame_;

  
  double start_time = ros::Time::now().toSec();
  traj_client_->sendGoal(goal);
  ROS_INFO("Sending goal to controller took %f seconds.", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToPose(double pose[6], double move_time)
{
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(quaternion, quaternion_msg);
  
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];
  pose_msg.orientation = quaternion_msg;

  std::vector<double> jnt_pos(7,0), solution(7,0);
  
  double start_time = ros::Time::now().toSec();
  if(computeIK(pose_msg,jnt_pos,solution))
  {
    ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
  }
  else
  {
	ROS_ERROR("IK FAILED!!!!");
    return;
  }
  
  double configuration[7];

  for(int i = 0; i < 7; i++)
    configuration[i] = solution[i];

  sendArmToConfiguration(configuration, move_time);
}

void Arm::sendArmToPoseQuaternion(double pose[7], double move_time)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];
  pose_msg.orientation.x = pose[3];
  pose_msg.orientation.y = pose[4];
  pose_msg.orientation.z = pose[5];
  pose_msg.orientation.w = pose[6];

  std::vector<double> jnt_pos(7,0), solution(7,0);

  double start_time = ros::Time::now().toSec();
  if(computeIK(pose_msg,jnt_pos,solution))
  {
    ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
  }
  else
    return;

  double configuration[7];

  for(int i = 0; i < 7; i++)
    configuration[i] = solution[i];

  sendArmToConfiguration(configuration, move_time);
}

void Arm::sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times)
{
  geometry_msgs::Pose pose_msg;
  btQuaternion quaternion;
  geometry_msgs::Quaternion quaternion_msg;
  std::vector<std::vector<double> > configurations(poses.size(), std::vector<double> (7,0));

  std::vector<double> jnt_pos(7,0), solution(7,0);

  for(unsigned int i =0; i < poses.size(); i++)
  {
    quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(quaternion, quaternion_msg);

    pose_msg.position.x = poses[i][0];
    pose_msg.position.y = poses[i][1];
    pose_msg.position.z = poses[i][2];
    pose_msg.orientation = quaternion_msg;

    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_msg,jnt_pos,configurations[i]))
      ROS_DEBUG("[sendArmToPoses] Computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    else
      return;
  }

  sendArmToConfigurations(configurations, move_times);
}

bool Arm::computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
  kinematics_msgs::GetPositionIK::Request request;
  kinematics_msgs::GetPositionIK::Response response;
  if(arm_name_.compare("left") == 0)
    request.ik_request.ik_link_name = "l_wrist_roll_link";
  else
    request.ik_request.ik_link_name = "r_wrist_roll_link";

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;
  request.timeout = ros::Duration(2.0);

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id = reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  vector<double> c_jangles;
  getCurrentArmConfiguration(c_jangles);

  request.ik_request.ik_seed_state.joint_state.position.clear();
  request.ik_request.ik_seed_state.joint_state.position = c_jangles;

  ROS_DEBUG("%f %f %f", pose.position.x,pose.position.y,pose.position.z);
  //for(int j = 0 ; j < 7; ++j)
  //  request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(ik_service_name_);
  ros::ServiceClient client = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name_, true);

  if(client.call(request, response))
  {
    ROS_DEBUG("Obtained IK solution");
    if(response.error_code.val == response.error_code.SUCCESS)
      for(unsigned int i=0; i < response.solution.joint_state.name.size(); i ++)
      {
        solution[i] = response.solution.joint_state.position[i];
        ROS_DEBUG("Joint: %s %f",response.solution.joint_state.name[i].c_str(),response.solution.joint_state.position[i]);
      }
    else
    {
      ROS_ERROR("Inverse kinematics failed for %s. (error code: %d)", request.ik_request.ik_link_name.c_str(), response.error_code.val);
      return false;
    }

    ROS_DEBUG("IK Solution");
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }

  return true;
}

bool Arm::performFK(const std::vector<double> jnt_pos, std::string frame, std::vector<double> &cart_pose)
{
  ROS_INFO("Started Forward Kinematics.");
  ros::service::waitForService(fk_service_name_);
  ros::ServiceClient fk_client = nh_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service_name_,true);

  // define the service messages
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;

  fk_request.header.frame_id = reference_frame_;
  fk_request.fk_link_names.clear();

  if(arm_name_.compare("left") == 0)
    fk_request.fk_link_names.push_back("l_wrist_roll_link");
  else
    fk_request.fk_link_names.push_back("r_wrist_roll_link");

  fk_request.robot_state.joint_state.name = joint_names_;
  fk_request.robot_state.joint_state.position.clear();
  fk_request.robot_state.joint_state.position = jnt_pos;

  if(fk_client.call(fk_request, fk_response))
  {

    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {

    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.x);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.y);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.z);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.x);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.y);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.z);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.w);
			ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[0].c_str());
			ROS_INFO_STREAM("Position: " <<
				fk_response.pose_stamped[0].pose.position.x << "," <<
				fk_response.pose_stamped[0].pose.position.y << "," <<
				fk_response.pose_stamped[0].pose.position.z);
			ROS_INFO("Orientation: %f %f %f %f",
				fk_response.pose_stamped[0].pose.orientation.x,
				fk_response.pose_stamped[0].pose.orientation.y,
				fk_response.pose_stamped[0].pose.orientation.z,
				fk_response.pose_stamped[0].pose.orientation.w);
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
    return false;
  }

  tf::StampedTransform transform;
  tf::Pose tf_pose;
  geometry_msgs::Pose pose_msg;
  // get transform to frame that collision map is in
  try
  {
    if(arm_name_.compare("left") == 0)
      tf_.lookupTransform("l_wrist_roll_link", frame, ros::Time(0), transform);
    else      
      tf_.lookupTransform("r_wrist_roll_link", frame, ros::Time(0), transform);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",joint_names_[6].c_str(),frame.c_str(), transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());

    tf::poseMsgToTF(fk_response.pose_stamped[0].pose, tf_pose);
    tf_pose = tf_pose*transform;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  tf::poseTFToMsg(tf_pose, pose_msg);
  cart_pose[0] = pose_msg.position.x;
  cart_pose[1] = pose_msg.position.y;
  cart_pose[2] = pose_msg.position.z;
  cart_pose[3] = pose_msg.orientation.x;
  cart_pose[4] = pose_msg.orientation.y;
  cart_pose[5] = pose_msg.orientation.z;
  cart_pose[6] = pose_msg.orientation.w;

  ROS_ERROR("[arm] Transforming pose from %s to %s. Transformed Pose: %0.3f %0.3f %0.3f", reference_frame_.c_str(), frame.c_str(), cart_pose[0],cart_pose[1],cart_pose[2]);

  /*
  geometry_msgs::PoseStamped tpose;
  tf_.transformPose(frame, fk_response.pose_stamped[0], tpose);
  cart_pose[0] = tpose.pose.position.x;
  cart_pose[1] = tpose.pose.position.y;
  cart_pose[2] = tpose.pose.position.z;
  cart_pose[3] = tpose.pose.orientation.x;
  cart_pose[4] = tpose.pose.orientation.y;
  cart_pose[5] = tpose.pose.orientation.z;
  cart_pose[6] = tpose.pose.orientation.w;

  ROS_ERROR("[arm] Transforming pose from %s to %s. Transformed Pose: %0.3f %0.3f %0.3f", reference_frame_.c_str(), frame.c_str(), cart_pose[0],cart_pose[1],cart_pose[2]);
  */

  ROS_INFO("Stopped Forward Kinematics.");
	return true;
}

bool Arm::performFK(const std::vector<double> jnt_pos, std::vector<double> &cart_pose)
{
// From http://www.ros.org/wiki/pr2_kinematics/Tutorials/Tutorial%203
/* We shouldn't need the kinematic solver info. */
//  ros::ServiceClient query_client = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_fk_solver_info");
//  ros::service::waitForService("pr2_right_arm_kinematics/get_fk_solver_info");
//  // define the service messages
//  kinematics_msgs::GetKinematicSolverInfo::Request request;
//  kinematics_msgs::GetKinematicSolverInfo::Response response;
//  if(query_client.call(request,response))
//  {
//    for(unsigned int i=0; i<response.kinematic_solver_info.joint_names.size(); i++)
//    {
//      ROS_DEBUG("Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str());
//    }
//  }
//  else
//  {
//    ROS_ERROR("Could not call query service");
//    return false;
//  }
  ROS_INFO("Started Forward Kinematics.");
  ros::service::waitForService(fk_service_name_);
  ros::ServiceClient fk_client = nh_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service_name_,true);

  // define the service messages
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;

  fk_request.header.frame_id = reference_frame_;
  fk_request.fk_link_names.clear();
  if(arm_name_.compare("left") == 0)
  	fk_request.fk_link_names.push_back("l_wrist_roll_link");
  else
  	fk_request.fk_link_names.push_back("r_wrist_roll_link");

  fk_request.robot_state.joint_state.name = joint_names_;
  fk_request.robot_state.joint_state.position.clear();
  fk_request.robot_state.joint_state.position = jnt_pos;

  if(fk_client.call(fk_request, fk_response))
  {

    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {

    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.x);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.y);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.position.z);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.x);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.y);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.z);
    	cart_pose.push_back(fk_response.pose_stamped[0].pose.orientation.w);
			ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[0].c_str());
			ROS_INFO_STREAM("Position: " <<
				fk_response.pose_stamped[0].pose.position.x << "," <<
				fk_response.pose_stamped[0].pose.position.y << "," <<
				fk_response.pose_stamped[0].pose.position.z);
			ROS_INFO("Orientation: %f %f %f %f",
				fk_response.pose_stamped[0].pose.orientation.x,
				fk_response.pose_stamped[0].pose.orientation.y,
				fk_response.pose_stamped[0].pose.orientation.z,
				fk_response.pose_stamped[0].pose.orientation.w);
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
    return false;
  }

  ROS_INFO("Stopped Forward Kinematics.");
	return true;
}

void Arm::getCurrentArmConfiguration(vector<double>& current_angles)
{
  //get a single message from the topic 'r_arm_controller/state' or 'l_arm_controller/state'
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>(controller_state_name_);

  //extract the joint angles from it
  current_angles = state_msg->actual.positions;
}

void Arm::getCurrentArmPose(vector<double>& cpose)
{
	// Get the current joint angles of the robot.
	vector<double> current_angles;
	getCurrentArmConfiguration(current_angles);

	// Perform forward kinematics to figure out what the arm pose is.
	vector<double> current_pose;
	performFK(current_angles, current_pose);

	cpose = current_pose;
}

bool Arm::parseTrajectoryFile(std::string filename, std::vector<std::vector<double> > &traj)
{
  FILE* file = fopen(filename.c_str(), "r");
  std::vector<double> v(7,0);

  if(file == NULL)
  {
    ROS_ERROR("ERROR: unable to open the file. Exiting.");
    return false;
  }

  while(!feof(file))
  {
    if(fscanf(file,"%lf %lf %lf %lf %lf %lf %lf ",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
      ROS_ERROR("Error parsing arm configuration pose.");
    else
      traj.push_back(v);
  }

  fclose(file);
  return true;
}

void Arm::printTrajectory(std::string name, std::vector<std::vector<double> > &traj)
{
  printf("[%s]\n" , name.c_str());
  for(size_t i = 0; i < traj.size(); ++i)
  {
    printf("% 2d: ",int(i));
    for(size_t j = 0; j < traj[i].size(); ++j)
      printf("% 0.4f ", traj[i][j]);
    printf("\n");
  }
}

void Arm::pathToJointTrajectory(std::vector<std::vector<double> > &path, std::vector<double> &time_from_start, trajectory_msgs::JointTrajectory &traj)
{
  if(path.size() != time_from_start.size())
  {
    ROS_ERROR("Can't convert path to JointTrajectory. (time_from_start.size: %d, path.size: %d)", int(time_from_start.size()), int(path.size()));
    return;
  }
  
  traj.header.seq = 0;
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = reference_frame_;
  traj.points.resize(path.size());

  for(size_t i = 0; i < 7; i++)
    traj.joint_names.push_back(joint_names_[i]);

  for(size_t i = 0; i < path.size(); i++)
  {
    traj.points[i].time_from_start = ros::Duration(time_from_start[i]);
    traj.points[i].positions.resize(7);
    //traj.points[i].velocities.resize(7);
    for(size_t j = 0; j < 7; j++){
      traj.points[i].positions[j] = path[i][j];
      //traj.points[i].velocities[j]=0.0;
    }
  }
}

bool Arm::filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, trajectory_msgs::JointTrajectory &trajectory_out)
{
  ros::ServiceClient filter_trajectory_client = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter/filter_trajectory_with_constraints");

  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;

  req.trajectory = trajectory_in;
  req.allowed_time = ros::Duration(1.0);
  ros::Time smoothing_time = ros::Time::now();
  req.group_name = group_name_;
  getRobotState(req.start_state);

  if(filter_trajectory_client.call(req,res))
  {
    ROS_INFO("Smoothing time is %0.3f", (ros::Time::now()-smoothing_time).toSec());
    trajectory_out = res.trajectory;
    return true;
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed.");
    return false;
  }
}

void Arm::getRobotState(arm_navigation_msgs::RobotState &state)
{
  sensor_msgs::JointStateConstPtr joint_state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
  state.joint_state = *joint_state;
}



/*
int main(int argc, char**argv)
{
  ros::init(argc,argv,"arm");

  Arm arm("right");
  double pose[6] = {0.7,-0.2,0.74, 0,0,0};

  arm.sendArmToPose(pose);

  sleep(15);

  return 0;
}
*/

