#include "wind_visualize.h"
#include <time.h>
#include <olfaction_msgs/anemometer.h>
#include <tf/transform_listener.h>

class SubscribeAndPublish {
	public:
		SubscribeAndPublish() {
			pub_ = n_.advertise<visualization_msgs::Marker>("WindSensor_display", 100);
			sub_ = n_.subscribe("/chatter", 200, &SubscribeAndPublish::windCallback, this);
			// sub_ = n_.subscribe("/Anemometer/WindSensor_reading", 200, &SubscribeAndPublish::windCallback, this);
		}

		void windCallback(const olfaction_msgs::anemometer::ConstPtr& windMsg) {
			visualization_msgs::Marker wind_point_inv;
			
			std::string	input_sensor_frame = "anemometer_frame";
			std::string	input_fixed_frame = "map";

			wind_point_inv.header.frame_id = input_sensor_frame.c_str();
			wind_point_inv.action = visualization_msgs::Marker::ADD;
			wind_point_inv.ns = "measured_wind_inverted";
			wind_point_inv.type = visualization_msgs::Marker::ARROW;

			
			bool know_sensor_pose = true;  

			//Get pose of the sensor in the /map reference
			try {
				listener.lookupTransform(input_fixed_frame.c_str(), input_sensor_frame.c_str(), ros::Time(0), transform);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
				know_sensor_pose = false;
				ros::Duration(1.0).sleep();
			}
			
			if (know_sensor_pose) {				
				//Current sensor pose
				float x_pos = transform.getOrigin().x();
				float y_pos = transform.getOrigin().y();
				float z_pos = transform.getOrigin().z();

				float downWind_direction_map = windMsg->wind_direction;
				double wind_speed = windMsg->wind_speed;
				// double wind_direction;
				
				// (IMPORTANT) Follow standards on wind measurement (real anemometers):
				//return the upwind direction in the anemometer reference system
				//range [-pi,pi]
				//positive to the right, negative to the left (opposed to ROS poses :s)

				// float upWind_direction_map = angles::normalize_angle(downWind_direction_map + 3.14159);

				// //Transform from map ref_system to the anemometer ref_system using TF
				// geometry_msgs::PoseStamped anemometer_upWind_pose, map_upWind_pose;
				// try {
				// 	map_upWind_pose.header.frame_id = input_fixed_frame.c_str();
				// 	map_upWind_pose.pose.position.x = 0.0;
				// 	map_upWind_pose.pose.position.y = 0.0;
				// 	map_upWind_pose.pose.position.z = 0.0;
				// 	map_upWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(upWind_direction_map);

				// 	tf_.transformPose(input_sensor_frame.c_str(), map_upWind_pose, anemometer_upWind_pose);
				// }
				// catch(tf::TransformException &ex) {
				// 	ROS_ERROR("FakeAnemometer - %s - Error: %s", __FUNCTION__, ex.what());
				// }

				// double upwind_direction_anemo = tf::getYaw(anemometer_upWind_pose.pose.orientation);
				// wind_direction = upwind_direction_anemo;
				
				//Add inverted wind marker --> DownWind
				wind_point_inv.header.stamp = ros::Time::now();
				wind_point_inv.header.frame_id=input_sensor_frame.c_str();
				wind_point_inv.points.clear();
				wind_point_inv.id = 1;  //unique identifier for each arrow
			
				wind_point_inv.pose.position.x = 0.0;
				wind_point_inv.pose.position.y = 0.0;
				wind_point_inv.pose.position.z = 0.0;
					
				// wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(wind_direction+3.1416);
				wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(windMsg->wind_direction);
				wind_point_inv.scale.x = wind_speed;	  //arrow lenght
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
};//End of class SubscribeAndPublish


int main( int argc, char** argv ) {
	ros::init(argc, argv, "real_anemometer");
	SubscribeAndPublish SAPObject;

	// n.param<std::string>("sensor_frame", input_sensor_frame, "/anemometer_link");	//sensor_frame
	// n.param<std::string>("fixed_frame", input_fixed_frame, "/map");	//fixed frame

    //What ref system to use for publishing measurements
    // n.param<bool>("use_map_ref_system", use_map_ref_system, false);

	// //Publishers & Subcriber
	// ros::Subscriber sub = n.subscribe("chatter", 200, windCallback);
	// ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("WindSensor_display", 100);
	
	ros::spin();
	return 0;
}