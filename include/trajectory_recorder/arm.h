#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/RobotState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <boost/thread.hpp>

using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Arm
{
  public: 
    
  Arm(std::string arm_name, ros::Subscriber sub);
    ~Arm();

    void stopArm();
    //void subscriberThread();
    // move wrist to {x,y,z,r,p,y}
    void sendArmToPose(double pose[], double move_time);
    void sendArmToPoseQuaternion(double pose[], double move_time);
    void sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times);

    void sendArmToConfiguration(double configuration[], double move_time);
    double time(double initial[], double final[], double move_time);
    void spinUntilWithinError(double expected[]);
    void armControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& message);
    void sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times);
    void sendArmToConfiguration(std::vector<float> configuration, double move_time);
    void sendArmToJointTrajectory(trajectory_msgs::JointTrajectory &traj);

    bool computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution);
    bool performFK(const std::vector<double> jnt_pos, std::vector<double> &cart_pose);
    bool performFK(const std::vector<double> jnt_pos, std::string frame, std::vector<double> &cart_pose);

    void getCurrentArmConfiguration(vector<double>& current_angles);
    void getCurrentArmPose(vector<double>& cpose);

    void printTrajectory(std::string name, std::vector<std::vector<double> > &traj);
    bool parseTrajectoryFile(std::string filename, std::vector<std::vector<double> > &traj);

    void pathToJointTrajectory(std::vector<std::vector<double> > &path, std::vector<double> &time_from_start, trajectory_msgs::JointTrajectory &traj);
    bool filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, trajectory_msgs::JointTrajectory &trajectory_out);
    void getRobotState(arm_navigation_msgs::RobotState &state);
    void setCurrentGoal(double goal[], bool usePrecision);
    double timePath(vector<vector<double> > points, vector<double> times);
    void cohereAngles(double angles[]);
    void makePr2Angles(double angles[]);
    void cohereAngles(vector<double> &angles);
    void makePr2Angles(vector<double> &angles);
    void printArray(char *s, const double a[7]);
    double *min;
    double *max;
 private:
    
    //ros::Subscriber joint_states_subscriber_;
    tf::TransformListener tf_;
    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    std::string arm_name_;
    std::string group_name_;
    std::string ik_service_name_;
    std::string fk_service_name_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_;
    volatile bool withinError;
    std::string controller_state_name_;
    
    double *current_goal;
    double start;
    double pos_thresh;
    double vel_thresh;
    bool started;
    double smallest;
    boost::thread tid;
    ros::MultiThreadedSpinner spinner;
    ros::AsyncSpinner *aspinner;
    boost::mutex mux;
    boost::mutex spin_once;
    bool *useError;
    TrajClient* traj_client_;
};


