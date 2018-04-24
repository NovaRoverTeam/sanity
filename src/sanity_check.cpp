#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <ros/console.h>
using namespace std;

#include <std_msgs/Empty.h>
#include <rover/RPM.h>
#include <rover/ReqRPM.h>
#include <rover/Voltages.h>


#define LOOP_HERTZ                  1
#define MAINFRAME_HERTZ             10
#define VOLTAGES_HERTZ              50
#define RPM_HERTZ                   50
#define MAX_TIME_WITHOUT_HBEAT      5
#define MAX_TIME_WITH_LOW_VOLTAGE   5
#define MAX_TIME_WITH_RPM_EXCEEDING 5
#define NUM_OF_BATTERIES            4

#define RPM_DIFFERENCE_THRESHOLD    10
#define MIN_VOLTAGE_THRESHOLD       10.0
const int HBEAT_TIMEOUT         =   MAX_TIME_WITHOUT_HBEAT * MAINFRAME_HERTZ;
const int VOLTAGES_TIMEOUT      =   MAX_TIME_WITH_LOW_VOLTAGE * VOLTAGES_HERTZ;
const int RPM_TIMEOUT           =   MAX_TIME_WITH_RPM_EXCEEDING * RPM_HERTZ;

bool num_of_batteries_found     =   false;

unsigned int rpm_count          =   0;
unsigned int hbeat_count        =   0;
unsigned int voltages_count     =   0;

float voltages[4]               =   {0.0,0.0,0.0,0.0};
bool batt_is_connected[4]       =   {1,1,1,1};
int req_rpm[4]                  =   {0,0,0,0};
int actual_rpm[4]               =   {0,0,0,0};

// _______________________________FUNCTIONS__________________________________
bool Num_Outside_Of_Range(int desired_num, int actual_num, int range_difference)
{
    bool below_min = (actual_num < (desired_num - range_difference));
    bool above_max = (actual_num > (desired_num + range_difference));

    return (below_min || above_max);
}

// _______________________________ROS CALLBACKS_______________________________
void voltage_cb(const rover::Voltages::ConstPtr& msg)
{
    voltages[0] = msg->v1;
    voltages[1] = msg->v2;
    voltages[2] = msg->v3;
    voltages[3] = msg->v4;    
    if(num_of_batteries_found) {
        bool v1_is_below_min = batt_is_connected[0] * (voltages[0] < MIN_VOLTAGE_THRESHOLD);
        bool v2_is_below_min = batt_is_connected[1] * (voltages[1] < MIN_VOLTAGE_THRESHOLD);
        bool v3_is_below_min = batt_is_connected[2] * (voltages[2] < MIN_VOLTAGE_THRESHOLD);
        bool v4_is_below_min = batt_is_connected[3] * (voltages[3] < MIN_VOLTAGE_THRESHOLD);

        if(v1_is_below_min || v2_is_below_min || v3_is_below_min || v4_is_below_min)    voltages_count++;
        else    voltages_count = 0;
    }
    else
    {
        for(int i=0; i<NUM_OF_BATTERIES; i++)
        {
            if(voltages[i] < 1.0)   batt_is_connected[i] = 0;
        }
        num_of_batteries_found = 1;
    }
}

void hbeat_cb(const std_msgs::Empty::ConstPtr& msg)
{
    hbeat_count = 0;
}

void reqRPM_cb(const rover::ReqRPM::ConstPtr& msg)
{
    req_rpm[0] = msg->req_rpm_fl;
    req_rpm[1] = msg->req_rpm_br;
    req_rpm[2] = msg->req_rpm_bl;
    req_rpm[3] = msg->req_rpm_fr;
}

void encoders_cb(const rover::RPM::ConstPtr& msg)
{
    actual_rpm[0] = msg->rpm_fl;
    actual_rpm[1] = msg->rpm_br;
    actual_rpm[2] = msg->rpm_bl;
    actual_rpm[3] = msg->rpm_fr;

    bool rpm_0_exceeds_thresh = Num_Outside_Of_Range(req_rpm[0], actual_rpm[0], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_1_exceeds_thresh = Num_Outside_Of_Range(req_rpm[1], actual_rpm[1], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_2_exceeds_thresh = Num_Outside_Of_Range(req_rpm[2], actual_rpm[2], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_3_exceeds_thresh = Num_Outside_Of_Range(req_rpm[3], actual_rpm[3], RPM_DIFFERENCE_THRESHOLD);

    if(rpm_0_exceeds_thresh || rpm_1_exceeds_thresh || rpm_2_exceeds_thresh || rpm_3_exceeds_thresh) rpm_count++;
    else rpm_count = 0;
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
    ros::Subscriber reqRPM_sub = n.subscribe("req_rpm", 4, reqRPM_cb);

    while(ros::ok())
    {
        if(hbeat_count > HBEAT_TIMEOUT)
        {
            ROS_ERROR_STREAM("NO HEARTBEAT DETECTED; REVERTING TO STANDBY MODE");
        }

        if(voltages_count > VOLTAGES_TIMEOUT)
        {
            ROS_ERROR_STREAM("BATTERY VOLTAGE DROPPED BELOW THRESHOLD; REVERTING TO STANDBY MODE. CURRENT BATTERY VOLTAGES - 1: " << voltages[0] << " 2: " << voltages[1] << " 3: " << voltages[2] << " 4: " << voltages[3]);
        }

        if(rpm_count > RPM_TIMEOUT)
        {
            ROS_ERROR_STREAM("RPM IS NOT WITHIN DESIRED RANGE; REVERTING TO STANDBY MODE. Current RPM VALUES (DESIRED|ACTUAL) - 1: (" << req_rpm[0] << "|" << actual_rpm[0] << ") 2: (" << req_rpm[1] << "|" << actual_rpm[1] << ") 3: (" << req_rpm[2] << "|" << actual_rpm[2] << ") 4: (" << req_rpm[3] << "|" << actual_rpm[3] << ")");

        }

        // Limit counter
        if(hbeat_count < (10*HBEAT_TIMEOUT)) hbeat_count++;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
