/**
 * @file gas_read.h
 *
 * @brief This code detects gas existance using ARX model. 
 *
 * @author Luong Duc Nhat
 * Contact: luong.d.aa@m.titech.ac.jp
 * 
 * @copyright Copyright 2021, The Bombyxbot 3.0 Project"
 * credits ["Luong Duc Nhat"]
 * license GPL
 * 
 * @version    = "1.0.0"
 * maintainer  = "Luong Duc Nhat"
 * status      = "Production"
 */

#ifndef GAS_READ_H
#define GAS_READ_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>

class Arx {
    public:
        Arx() {}
        Arx(std::string publicTopicName1, std::string publicTopicName2, std::string SubscribeTopicName, int queueSize) {
            PublisherObj1 = nH.advertise<std_msgs::String>(publicTopicName1,queueSize);
            PublisherObj2 = nH.advertise<std_msgs::Int32>(publicTopicName2,queueSize);
            SubscriberObj = nH.subscribe<gas_intake_msgs::Gas>(SubscribeTopicName, queueSize, &Arx::SubscriberCallback,this);
        }

        void SubscriberCallback(const typename gas_intake_msgs::Gas::ConstPtr& recieveMsg);
        void shift_array(double* val_array, int num);
        double cal_average(double* val_array, int num);
        void getArxValues(double* raw_val);
        int getStimuli();

    protected:
        ros::Subscriber SubscriberObj;
        ros::Publisher PublisherObj1, PublisherObj2;
        ros::NodeHandle nH;
        
        // ARX constants
        const double a1 = -0.981;
        const double a2 = 0.01653;
        const double b0 = 0.2833;
        const double b1 = -0.2706;
        
        const double TIME_STEP = 0.02;   //second
        const double THRESHOLD = 0.04;
        const double SP_THRESHOLD = 0;

        // Settings of average movement filter
        static const int yNUM    = 7;
        static const int ma_yNUM = 3;
        static const int dyNUM   = 2;
        static const int uNUM    = 7;
        static const int ma_uNUM = 5;
        static const int N_SENSOR  = 3; //number of gas sensors

        double y[N_SENSOR][yNUM] = {0};  //Sensor value history
        double u[N_SENSOR][uNUM] = {0};
        double dy[N_SENSOR][dyNUM] = {0};
        double ma_y[N_SENSOR][ma_yNUM] = {0};  //Moving average gas sensing
        double ma_u[N_SENSOR][ma_uNUM] = {0};   
        ros::Time begin = ros::Time::now();
};

#endif