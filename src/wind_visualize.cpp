#include <time.h>
#include <olfaction_msgs/anemometer.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <angles/angles.h>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>


class SubscribeAndPublish {
	public:
		SubscribeAndPublish() {
			pub_ = n_.advertise<visualization_msgs::Marker>("WindSensor_display", 100);
			sub_ = n_.subscribe("/chatter", 200, &SubscribeAndPublish::windCallback, this);
			input_sensor_frame = "anemometer_frame";
			input_fixed_frame = "map";
		}

		void windCallback(const olfaction_msgs::anemometer::ConstPtr& msg) {
			double reading_speed = msg->wind_speed;          // (m/s)
			double reading_direction = msg->wind_direction;  // (rad) This is the Upwind direction with respect the Anemometer ref system (standard)
			float downwind_direction_map;

			// //Transform Upwind direction in the Anemometer ref system to downWind direction in the MAP ref system
			// if (reading_speed != 0.0) {
			// 	//Transform from anemometer ref_system to the map ref_system using TF
			// 	geometry_msgs::PoseStamped anemometer_upWind_pose, map_upWind_pose;
			// 	try {
			// 		anemometer_upWind_pose.header.frame_id = msg->header.frame_id.c_str();  //id: anemometer
			// 		anemometer_upWind_pose.pose.position.x = 0.0;
			// 		anemometer_upWind_pose.pose.position.y = 0.0;
			// 		anemometer_upWind_pose.pose.position.z = 0.0;
			// 		anemometer_upWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(reading_direction);

			// 		//lookuptransform (target_frame, target_time, pose_in, fixed_frame, pose_out)
			// 		tf_listener.transformPose(input_fixed_frame.c_str(), anemometer_upWind_pose, map_upWind_pose);
			// 		downwind_direction_map = angles::normalize_angle(tf::getYaw(map_upWind_pose.pose.orientation) + 3.14159);
			// 	}
			// 	catch(tf::TransformException &ex) {
			// 		ROS_ERROR("%s - Error: %s", __FUNCTION__, ex.what());
			// 	}
			// }
			// else {
			// 	downwind_direction_map = 0.0;
			// }
				
			// // Get pose of the sensor in the /map reference system
			// tf::StampedTransform transform; 
			// try {
			// 	//lookuptransform (target_frame, source_frame, result_tf)
			// 	//time 0 means "the latest available" transform in the buffer. 
			// 	tf_listener.lookupTransform(input_fixed_frame.c_str(),msg->header.frame_id.c_str(), ros::Time(0), transform);
			// }
			// catch (tf::TransformException ex) {
			// 	ROS_ERROR("[GMRF] exception when reading observation: %s",ex.what());
			// 	ros::Duration(1.0).sleep();
			// }
			

			bool know_sensor_pose = true;  
			
			if (know_sensor_pose) {				
			// 	//Current sensor pose
			// 	float x_pos = transform.getOrigin().x();
			// 	float y_pos = transform.getOrigin().y();
			// 	float z_pos = transform.getOrigin().z();

			// 	float downWind_direction_map = windMsg->wind_direction;
			// 	double wind_speed = windMsg->wind_speed;
			// 	double wind_direction;
				
			// 	// (IMPORTANT) Follow standards on wind measurement (real anemometers):
			// 	//return the upwind direction in the anemometer reference system
			// 	//range [-pi,pi]
			// 	//positive to the right, negative to the left (opposed to ROS poses :s)

			// 	float upWind_direction_map = angles::normalize_angle(downWind_direction_map + 3.14159);

			// 	//Transform from map ref_system to the anemometer ref_system using TF
			// 	geometry_msgs::PoseStamped anemometer_upWind_pose, map_upWind_pose;
			// 	try {
			// 		map_upWind_pose.header.frame_id = input_fixed_frame.c_str();
			// 		map_upWind_pose.pose.position.x = 0.0;
			// 		map_upWind_pose.pose.position.y = 0.0;
			// 		map_upWind_pose.pose.position.z = 0.0;
			// 		map_upWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(upWind_direction_map);

			// 		tf_.transformPose(input_sensor_frame.c_str(), map_upWind_pose, anemometer_upWind_pose);
			// 	}
			// 	catch(tf::TransformException &ex) {
			// 		ROS_ERROR("FakeAnemometer - %s - Error: %s", __FUNCTION__, ex.what());
			// 	}

			// 	double upwind_direction_anemo = tf::getYaw(anemometer_upWind_pose.pose.orientation);
			// 	wind_direction = upwind_direction_anemo;
				
				//Add inverted wind marker --> DownWind

				wind_point_inv.header.frame_id = input_sensor_frame.c_str();
				wind_point_inv.action = visualization_msgs::Marker::ADD;
				wind_point_inv.ns = "measured_wind_inverted";
				wind_point_inv.type = visualization_msgs::Marker::ARROW;


				wind_point_inv.header.stamp = ros::Time::now();
				wind_point_inv.header.frame_id=input_sensor_frame.c_str();
				wind_point_inv.points.clear();
				wind_point_inv.id = 1;  //unique identifier for each arrow
			
				wind_point_inv.pose.position.x = 0.0;
				wind_point_inv.pose.position.y = 0.0;
				wind_point_inv.pose.position.z = 0.0;
					
				wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(reading_direction+3.1416);
				wind_point_inv.scale.x = reading_speed;	  //arrow lenght
				wind_point_inv.scale.y = 0.1;	  //arrow width
				wind_point_inv.scale.z = 0.1;	  //arrow height
				wind_point_inv.color.r = 0.0;
				wind_point_inv.color.g = 0.0;
				wind_point_inv.color.b = 1.0;
				wind_point_inv.color.a = 1.0;
				pub_.publish(wind_point_inv);
			}			
		}

	private:
		tf::TransformListener listener;
		tf::TransformListener tf_;
		tf::StampedTransform transform;
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Subscriber sub_;
		std::string	input_sensor_frame;
		std::string	input_fixed_frame;
		visualization_msgs::Marker wind_point_inv;
};


int main( int argc, char** argv ) {
	ros::init(argc, argv, "real_anemometer");
	SubscribeAndPublish SAPObject;	
	ros::spin();
	return 0;
}