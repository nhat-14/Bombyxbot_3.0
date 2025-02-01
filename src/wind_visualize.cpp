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

class SubscribeAndPublish {
	public:
		SubscribeAndPublish() {
			pub_ = n_.advertise<visualization_msgs::Marker>("WindSensor_display", 100);
			sub_ = n_.subscribe("/windTopic", 200, &SubscribeAndPublish::windCallback, this);
			input_sensor_frame = "anemometer_frame";
			input_fixed_frame = "map";

			wind_point_inv.action = visualization_msgs::Marker::ADD;
			wind_point_inv.ns = "measured_wind_inverted";
		    wind_point_inv.type = visualization_msgs::Marker::ARROW;
		}

		void windCallback(const olfaction_msgs::anemometer::ConstPtr& msg) {
			double reading_speed = msg->wind_speed;          // (m/s)
			double reading_direction = msg->wind_direction;  // (rad) This is the Upwind direction with respect the Anemometer ref system (standard)
			float downwind_direction_map;      
						
            wind_point_inv.header.stamp = ros::Time::now();
            wind_point_inv.header.frame_id = input_sensor_frame.c_str();
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