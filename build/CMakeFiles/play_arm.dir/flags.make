# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# compile CXX with /usr/bin/c++
CXX_FLAGS = -O2 -g -I/home/wmarshall/workspace/trajectory_recorder/include -I/opt/ros/electric/stacks/navigation/move_base/include -I/opt/ros/electric/stacks/navigation/move_base_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/navigation/navfn/include -I/opt/ros/electric/stacks/navigation/navfn/cfg/cpp -I/opt/ros/electric/stacks/navigation/navfn/srv_gen/cpp/include -I/opt/ros/electric/stacks/navigation/clear_costmap_recovery/include -I/opt/ros/electric/stacks/navigation/rotate_recovery/include -I/opt/ros/electric/stacks/navigation/base_local_planner/include -I/opt/ros/electric/stacks/navigation/base_local_planner/cfg/cpp -I/opt/ros/electric/stacks/navigation/base_local_planner/msg_gen/cpp/include -I/opt/ros/electric/stacks/navigation/costmap_2d/cfg/cpp -I/opt/ros/electric/stacks/navigation/costmap_2d/include -I/opt/ros/electric/stacks/navigation/costmap_2d/msg_gen/cpp/include -I/opt/ros/electric/stacks/laser_pipeline/laser_geometry/include -I/opt/ros/electric/stacks/navigation/map_server/include -I/usr/include/yaml-cpp -I/opt/ros/electric/stacks/navigation/voxel_grid/include -I/opt/ros/electric/stacks/navigation/nav_core/include -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/include -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/cfg/cpp -I/opt/ros/electric/stacks/nodelet_core/nodelet_topic_tools/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/srv_gen/cpp/include -I/opt/ros/electric/stacks/bond_core/bondcpp/include -I/opt/ros/electric/stacks/bond_core/bond/msg_gen/cpp/include -I/opt/ros/electric/stacks/bond_core/smclib/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv_gen/cpp/include -I/opt/ros/electric/stacks/perception_pcl/pcl/include/pcl-1.1 -I/usr/include/vtk-5.2 -I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -I/usr/include/tcl8.5 -I/usr/include/python2.6 -I/usr/lib/jvm/default-java/include -I/usr/include/libxml2 -I/usr/include/freetype2 -I/opt/ros/electric/stacks/perception_pcl/pcl/msg_gen/cpp/include -I/opt/ros/electric/stacks/perception_pcl/cminpack/include -I/opt/ros/electric/stacks/perception_pcl/flann/include -I/opt/ros/electric/stacks/pr2_controllers/joint_trajectory_action/msg/cpp -I/opt/ros/electric/stacks/common/actionlib/include -I/opt/ros/electric/stacks/common/actionlib/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/joystick_drivers/joy/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_action/msg/cpp -I/opt/ros/electric/stacks/pr2_controllers/pr2_mechanism_controllers/include -I/opt/ros/electric/stacks/pr2_controllers/pr2_mechanism_controllers/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_controllers/pr2_mechanism_controllers/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/nav_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/nav_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/pr2_common/pr2_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_common/pr2_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_controllers/robot_mechanism_controllers/include -I/opt/ros/electric/stacks/pr2_controllers/robot_mechanism_controllers/msg/cpp -I/opt/ros/electric/stacks/pr2_controllers/robot_mechanism_controllers/srv/cpp -I/opt/ros/electric/stacks/pr2_controllers/robot_mechanism_controllers/msg_gen/cpp/include -I/opt/ros/electric/stacks/filters/include -I/opt/ros/electric/stacks/geometry/eigen_conversions/include -I/opt/ros/electric/stacks/pr2_mechanism/pr2_controller_manager/include -I/opt/ros/electric/stacks/diagnostics/diagnostic_updater/include -I/opt/ros/electric/stacks/pr2_mechanism/pr2_mechanism_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_mechanism/pr2_mechanism_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/diagnostic_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/diagnostic_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/robot_model/robot_state_publisher/include -I/opt/ros/electric/stacks/geometry/tf_conversions/include -I/usr/include/eigen3 -I/opt/ros/electric/stacks/pr2_controllers/control_toolbox/include -I/opt/ros/electric/stacks/pr2_controllers/control_toolbox/include/control_toolbox/eigen2 -I/opt/ros/electric/stacks/pr2_controllers/control_toolbox/srv_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/include -I/opt/ros/electric/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/control/control_msgs/include -I/opt/ros/electric/stacks/control/control_msgs/msg/cpp -I/opt/ros/electric/stacks/control/control_msgs/srv/cpp -I/opt/ros/electric/stacks/control/control_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_controller/msg/cpp -I/opt/ros/electric/stacks/pr2_mechanism/pr2_controller_interface/include -I/opt/ros/electric/stacks/pr2_mechanism/pr2_mechanism_model/include -I/opt/ros/electric/stacks/pr2_mechanism/pr2_hardware_interface/include -I/opt/ros/electric/stacks/robot_model/kdl_parser/include -I/opt/ros/electric/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/pr2_controllers/pr2_controllers_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/pr2_mechanism/realtime_tools/include -I/opt/ros/electric/stacks/pluginlib/include -I/opt/ros/electric/stacks/pluginlib -I/opt/ros/electric/stacks/ros_realtime/rosrt/include -I/opt/ros/electric/stacks/ros_realtime/lockfree/include -I/opt/ros/electric/stacks/ros_realtime/rosatomic/include -I/opt/ros/electric/stacks/ros_realtime/allocators/include -I/opt/ros/electric/stacks/ros_comm/messages/std_srvs/srv_gen/cpp/include -I/opt/ros/electric/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/cpp -I/opt/ros/electric/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/actionlib_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/trajectory_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/bullet/include -I/opt/ros/electric/stacks/geometry/angles/include -I/opt/ros/electric/stacks/ros_comm/utilities/message_filters/include -I/opt/ros/electric/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/include -I/opt/ros/electric/stacks/robot_model/urdf/include -I/opt/ros/electric/stacks/robot_model/urdf_parser/include -I/opt/ros/electric/stacks/robot_model/collada_parser/include -I/opt/ros/electric/stacks/robot_model/urdf_interface/include -I/opt/ros/electric/stacks/robot_model/colladadom/include -I/opt/ros/electric/stacks/robot_model/colladadom/include/1.5 -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/tools/rosbag/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/srv_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/tools/rostest/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include -I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src -I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include -I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include -I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include -I/opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/include -I/opt/ros/electric/ros/tools/rospack -I/opt/ros/electric/ros/tools/rospack/include -I/home/wmarshall/workspace/trajectory_recorder/msg_gen/cpp/include -I/home/wmarshall/workspace/trajectory_recorder/srv_gen/cpp/include   -DROS_PACKAGE_NAME='"trajectory_recorder"'

CXX_DEFINES = 

# TARGET_FLAGS = -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread

