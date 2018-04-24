#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <ros/console.h>
using namespace std;

#include <std_msgs/Empty.h>
#include <rover/RPM.h>
#include <rover/Voltages.h>


#define LOOP_HERTZ                  1
#define MAINFRAME_HERTZ             10
#define VOLTAGES_HERTZ              50
#define MAX_TIME_WITHOUT_HBEAT      5
#define MAX_TIME_WITH_LOW_VOLTAGE   5

#define RPM_DIFFERENCE_THRESHOLD    10
#define MIN_VOLTAGE_THRESHOLD       10.0
const int hbeat_timeout = MAX_TIME_WITHOUT_HBEAT * MAINFRAME_HERTZ;
const int voltages_timeout = MAX_TIME_WITH_LOW_VOLTAGE * VOLTAGES_HERTZ;

unsigned int hbeat_count = 0;
unsigned int voltages_count = 0;

float voltages[4] = {0.0,0.0,0.0,0.0};
int actual_RPM[4] = {0,0,0,0};


// _______________________________ROS CALLBACKS_______________________________
void voltage_cb(const rover::Voltages::ConstPtr& msg)
{
    voltages[0] = msg->v1;
    voltages[1] = msg->v2;
    voltages[2] = msg->v3;
    voltages[3] = msg->v4;    

    bool v1_is_below_min = (voltages[0] < MIN_VOLTAGE_THRESHOLD);
    bool v2_is_below_min = (voltages[1] < MIN_VOLTAGE_THRESHOLD);
    bool v3_is_below_min = (voltages[2] < MIN_VOLTAGE_THRESHOLD);
    bool v4_is_below_min = (voltages[3] < MIN_VOLTAGE_THRESHOLD);

    if(v1_is_below_min || v2_is_below_min || v3_is_below_min || v4_is_below_min) 
    {
        voltages_count++;
    }
    else 
    {
        voltages_count = 0;
    }
}

void hbeat_cb(const std_msgs::Empty::ConstPtr& msg)
{
    hbeat_count = 0;
}

void encoders_cb(const rover::RPM::ConstPtr& msg)
{
    actual_RPM[0] = msg->rpm_fl;
    actual_RPM[1] = msg->rpm_br;
    actual_RPM[2] = msg->rpm_bl;
    actual_RPM[3] = msg->rpm_fr;
}

// _________________________________MAIN_______________________________________
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sanity_check");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_HERTZ);

    ros::Subscriber encoders_sub = n.subscribe("encoders", 5, encoders_cb);
    ros::Subscriber hbeat_sub = n.subscribe("hbeat", 1, hbeat_cb);
    ros::Subscriber voltage_sub = n.subscribe("voltage", 1, voltage_cb);

    while(ros::ok())
    {
        if(hbeat_count > hbeat_timeout)
        {
            ROS_ERROR_STREAM("NO HEARTBEAT DETECTED; REVERTING TO STANDBY MODE");
        }

        if(voltages_count > voltages_timeout)
        {
            ROS_ERROR_STREAM("BATTERY VOLTAGE DROPPED BELOW THRESHOLD; REVERTING TO STANDBY MODE. CURRENT BATTERY VOLTAGES - 1: " << voltages[0] << " 2: " << voltages[1] << " 3: " << voltages[2] << " 4: " << voltages[3]);
        }

        // Limit counter
        if(hbeat_count < (10*hbeat_timeout)) hbeat_count++;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
