#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <angles/angles.h>

#include <cstdlib>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>

// Sensor Parameters
std::string		input_sensor_frame;
std::string		input_fixed_frame;
bool            use_map_ref_system;

//functions:
void  loadNodeParameters(ros::NodeHandle private_nh);