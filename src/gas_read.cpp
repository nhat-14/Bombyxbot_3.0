/**
 * @file gas_read.cpp
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

#include <ros/ros.h>
#include <gas_intake_msgs/Gas.h>
#include <std_msgs/String.h>
#include <gas_read.h>
#include <std_msgs/Int32.h>

void Arx::shift_array(double* val_array, int num) {
    for(int i = num-2; i >= 0; i--) {
        val_array[i+1] = val_array[i];
    }
}


double Arx::cal_average(double* val_array, int num) {
    double sum = 0.0;
    for(int i = 0; i < num; i++) {
        sum += val_array[i];
    }
    return sum/num;
}   


void Arx::getArxValues(double *raw_val) {
    //Update moving average gas sensing
    for (int i=0; i<N_SENSOR; i++) {
        shift_array(y[i], yNUM);
        y[i][0] = raw_val[i]; 
        
        shift_array(ma_y[i], ma_yNUM);
        ma_y[i][0] = cal_average(y[i], yNUM);

        for(int j = ma_yNUM-2; j >= 0; j--) {
            dy[i][j] = (ma_y[i][j] - ma_y[i][j+1])/TIME_STEP;
        }

        shift_array(u[i], uNUM);
        u[i][0] = -a1*u[i][1] - a2*u[i][2] + b0*dy[i][0] + b1*dy[i][1]; // ARX equation

        shift_array(ma_u[i], ma_uNUM);
        ma_u[i][0] = cal_average(u[i], uNUM);
    }
}        


int Arx::getStimuli() {
    int spike[N_SENSOR] = {0};
    int stumli[N_SENSOR] = {0};
    
    // Take few samples of the ARX model output and update the spike counters
    for(int i=0; i<N_SENSOR; i++) {
        for(int j=0; j<5; j++) {
            if(ma_u[i][j] > THRESHOLD) {
                spike[i]++;
            }
        }
        // Update stimuli outputs and directions then feedback to the server
        if(spike[i] > SP_THRESHOLD) {
            stumli[i] = 1;
        }
        else {
            stumli[i] = 0;
        }
    }

    // int hit_side = stumli[0]*100 + stumli[1]*10 + stumli[2];
    int hit_side = stumli[0] * 100 + stumli[1] * 0 + stumli[2];
    return hit_side;
}


void Arx::SubscriberCallback(const gas_intake_msgs::Gas::ConstPtr& recieveMsg) {
    double raw_val[N_SENSOR] = {recieveMsg->left, recieveMsg->front, recieveMsg->right};
    getArxValues(raw_val);
    std_msgs::String echo_msg;
    std_msgs::Int32 hit_side;

    ros::Time now = ros::Time::now();
    if ( ((now-begin).toSec()) > 5) {
        hit_side.data = getStimuli();

        switch (hit_side.data) {
            case 0:
                echo_msg.data = "nothing";
                break;
            case 1:
                echo_msg.data = "right";
                break;
            case 10:
                echo_msg.data = "both";
                break;
            case 11:
                echo_msg.data = "right";
                break;
            case 100:
                echo_msg.data = "left";
                break;
            case 101:
                echo_msg.data = "both";
                break;
            case 110:
                echo_msg.data = "left";
                break;
            case 111:
                echo_msg.data = "both";
                break;
            default:
                echo_msg.data = "nothing";
            }

        PublisherObj1.publish(echo_msg);    
        PublisherObj2.publish(hit_side); 
    }
     
}

//============================= MAIN ====================================
int main(int argc, char **argv) {
    ros::init(argc, argv, "gas_read");
    ros::NodeHandle nh;    
    Arx arx("arx", "gas_detect_led", "gas_read", 50);
    ros::spin();
    return 0;
}