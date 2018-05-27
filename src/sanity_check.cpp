/****************************************************************************************
 *  
 *   ROVER SANITY CHECKER - NOVA ROVER TEAM - URC2018
 *   This is the code executed onboard the rover to check that core functionalities are 
 *   operating within expected ranges. If a range is exceeded for a specified length of
 *   time, the rover will be set to standby mode.
 * 
 *   Author: Andrew Stuart
 *****************************************************************************************/

// ROS and standard file includes
#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <ros/console.h>
using namespace std;

// msg file includes
#include <std_msgs/Empty.h> // Heartbeat msg
#include <rover/RPM.h>
#include <rover/ReqRPM.h>
#include <rover/Voltages.h>
#include <gps/Gps.h>
#include <autopilot/calc_route.h>


#define LOOP_HERTZ                  1
#define MAINFRAME_HERTZ             1
#define VOLTAGES_HERTZ              1
#define RPM_HERTZ                   1
#define MAX_TIME_WITHOUT_HBEAT      3
#define MAX_TIME_WITH_LOW_VOLTAGE   5
#define MAX_TIME_WITH_RPM_EXCEEDING 300
#define NUM_OF_BATTERIES            4
#define RTB_TIME                    20 // Time until return to base

// Threshold values
#define RPM_DIFFERENCE_THRESHOLD    10
#define MIN_VOLTAGE_THRESHOLD       10.0

// Home position (MDRS)
#define MDRS_LAT    38.406447
#define MDRS_LONG -110.791943

// NodeHandle must be global
ros::NodeHandle* n;
ros::ServiceClient start_auto_client;

// Maximum time constant calculations
const int HBEAT_TIMEOUT    = MAX_TIME_WITHOUT_HBEAT * MAINFRAME_HERTZ;
const int VOLTAGES_TIMEOUT = MAX_TIME_WITH_LOW_VOLTAGE * VOLTAGES_HERTZ;
const int RPM_TIMEOUT      = MAX_TIME_WITH_RPM_EXCEEDING * RPM_HERTZ;
const int RTB_TIMEOUT      = RTB_TIME * MAINFRAME_HERTZ;

// The number of batteries must be determined before measuring the sensor voltage
bool num_of_batteries_found     =   false;

// The total times an error has accumulalted
unsigned int rpm_count          =   0;
unsigned int hbeat_count        =   0;
unsigned int voltages_count     =   0;

float voltages[4]               =   {0.0,0.0,0.0,0.0};
bool batt_is_connected[4]       =   {1,1,1,1};
int req_rpm[4]                  =   {0,0,0,0};
int actual_rpm[4]               =   {0,0,0,0};

gps::Gps home_coord; // Coordinate of the MDRS
bool rtb_engaged = false;

// _______________________________FUNCTIONS__________________________________
bool Num_Outside_Of_Range(int desired_num, int actual_num, int range_difference)
/****************************************************************************
 * This function checks to see if a number is within the range of a desired
 * number (+/- a threshold value). 
 *
 * Returns true if the number is outside the desired range.
 ****************************************************************************/
{
    bool below_min = (actual_num < (desired_num - range_difference));
    bool above_max = (actual_num > (desired_num + range_difference));

    return (below_min || above_max);
}

void Set_Mode_To_Standby()
/****************************************************************************
 * This function sets the rover state to STANDBY when a core functionality is
 * misbehaving. 
 ****************************************************************************/
{
    n->setParam("/STATE", "STANDBY");
}

void Print_Voltage_Error()
/****************************************************************************
 * Prints an error message to the ROS console with the voltages for each 
 * battery.
 * ***************************************************************************/
{
    ROS_ERROR_STREAM("BATTERY VOLTAGE DROPPED BELOW THRESHOLD; REVERTING TO STANDBY MODE. CURRENT BATTERY VOLTAGES - 1: " << voltages[0] << " 2: " << voltages[1] << " 3: " << voltages[2] << " 4: " << voltages[3]);
}

void Print_RPM_Error()     
/****************************************************************************
 * Prints an error message to the ROS console with the desired RPM and actual
 * RPM values for each wheel.
******************************************************************************/
{
    ROS_ERROR_STREAM("RPM IS NOT WITHIN DESIRED RANGE; REVERTING TO STANDBY MODE. Current RPM VALUES (DESIRED|ACTUAL) - 1: (" << req_rpm[0] << "|" << actual_rpm[0] << ") 2: (" << req_rpm[1] << "|" << actual_rpm[1] << ") 3: (" << req_rpm[2] << "|" << actual_rpm[2] << ") 4: (" << req_rpm[3] << "|" << actual_rpm[3] << ")");
}

// _______________________________ROS CALLBACKS_______________________________
void voltage_cb(const rover::Voltages::ConstPtr& msg)
/****************************************************************************
 * The ROS callback function for the voltage sensor message. This function 
 * is called every time a new voltage message is broadcast. 
 *
 * This function checks if the battery voltages have dropped below a minimum
 * voltage and, if so, increments an accumulator. If the accumulator value 
 * becomes too high, the system is set to STANDBY.
******************************************************************************/
{
    voltages[0] = msg->v1;
    voltages[1] = msg->v2;
    voltages[2] = msg->v3;
    voltages[3] = msg->v4;

    // Check for low voltage after the number of batteries has been found 
    if(num_of_batteries_found) {
        // For each connected battery, check if the battery's respective voltage is below the minimum.
        bool v1_is_below_min = batt_is_connected[0] * (voltages[0] < MIN_VOLTAGE_THRESHOLD);
        bool v2_is_below_min = batt_is_connected[1] * (voltages[1] < MIN_VOLTAGE_THRESHOLD);
        bool v3_is_below_min = batt_is_connected[2] * (voltages[2] < MIN_VOLTAGE_THRESHOLD);
        bool v4_is_below_min = batt_is_connected[3] * (voltages[3] < MIN_VOLTAGE_THRESHOLD);
        
        // If the voltage is below the minimum value, increment accumulator
        if(v1_is_below_min || v2_is_below_min || v3_is_below_min || v4_is_below_min)    voltages_count++;
        // Otherwise, reset it
        else    voltages_count = 0;
    }
    // Get the number of batteries and the slots they are connected to
    else
    {
        for(int i=0; i<NUM_OF_BATTERIES; i++)
        {
            // Use >1V as an indicator of a battery.
            if(voltages[i] < 1.0)   batt_is_connected[i] = 0;
        }
        num_of_batteries_found = 1;
    }
}

void hbeat_cb(const std_msgs::Empty::ConstPtr& msg)
/****************************************************************************
 * The ROS callback function for the heartbeat message. This function
 * is called every time a new heartbeat is broadcast from the mainframe.
 *
 * This functions ichecks to ensure a connection is being maintained from the
 * mainframe to the rover by ensuring "heartbeats" from the mainframe are
 * being received. If the heartbeat signal is lost for a specified amount of
 * time, the system is set to STANDBY. 
 ****************************************************************************/
{
    // Reset heartbeat accumulator
    hbeat_count = 0;
}

void reqRPM_cb(const rover::ReqRPM::ConstPtr& msg)
/****************************************************************************
 * The ROS callback function for the desired RPM message. This function
 * is called everytime a new RPM for the wheels is desired.
 *
 * This function saves the desired RPM values into an array which may be
 * accessed by other functions in the program. 
 ****************************************************************************/
{
    req_rpm[0] = msg->req_rpm_fl;
    req_rpm[1] = msg->req_rpm_br;
    req_rpm[2] = msg->req_rpm_bl;
    req_rpm[3] = msg->req_rpm_fr;
}

void encoders_cb(const rover::RPM::ConstPtr& msg)
/****************************************************************************
 * The ROS callback function for the encoders message. This function
 * is called every time new RPM values have been measured for the wheels.
 * 
 * This function uses the desired RPM values and actual RPM values for each
 * wheel to determine if the wheel speeds are not performing within an 
 * expected range. 
 ****************************************************************************/
{
    actual_rpm[0] = msg->rpm_fl;
    actual_rpm[1] = msg->rpm_br;
    actual_rpm[2] = msg->rpm_bl;
    actual_rpm[3] = msg->rpm_fr;

    // Check if each wheel speed is outside the threshold range (desired RPM +/- THRESHOLD)
    bool rpm_0_exceeds_thresh = Num_Outside_Of_Range(req_rpm[0], actual_rpm[0], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_1_exceeds_thresh = Num_Outside_Of_Range(req_rpm[1], actual_rpm[1], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_2_exceeds_thresh = Num_Outside_Of_Range(req_rpm[2], actual_rpm[2], RPM_DIFFERENCE_THRESHOLD);
    bool rpm_3_exceeds_thresh = Num_Outside_Of_Range(req_rpm[3], actual_rpm[3], RPM_DIFFERENCE_THRESHOLD);

    // If the RPM is outside the range, increment accumulator
    if(rpm_0_exceeds_thresh || rpm_1_exceeds_thresh || rpm_2_exceeds_thresh || rpm_3_exceeds_thresh) rpm_count++;
    // Otherwise, reset accumulator
    else rpm_count = 0;
}



// _________________________________MAIN_______________________________________
int main(int argc, char **argv)
{
    // ROS setup and iinitialisation
    ros::init(argc, argv, "sanity_check");
    n = new ros::NodeHandle("~");

    ros::Rate loop_rate(LOOP_HERTZ);
    ros::Subscriber encoders_sub = n->subscribe("/encoders", 5, encoders_cb);
    ros::Subscriber hbeat_sub = n->subscribe("/hbeat", 1, hbeat_cb);
    ros::Subscriber voltage_sub = n->subscribe("/voltage", 1, voltage_cb);
    ros::Subscriber reqRPM_sub = n->subscribe("/req_rpm", 4, reqRPM_cb);

    start_auto_client = 
      n->serviceClient<autopilot::calc_route>("/Start_Auto");
      
    home_coord.latitude =  MDRS_LAT;
    home_coord.longitude = MDRS_LONG;

    ros::Duration(5).sleep(); // Give sensors time to get values

    while(ros::ok())
    {
        string state; n->getParam("/STATE", state);

        // Only set to standby if no heartbeat, not returning to base and not in autonomous state
        if((hbeat_count > HBEAT_TIMEOUT) && (hbeat_count <= RTB_TIMEOUT) && (state != "AUTO"))
        {
            ROS_ERROR_STREAM("NO HEARTBEAT DETECTED; REVERTING TO STANDBY MODE");
            Set_Mode_To_Standby();
            
            rtb_engaged = false;
        }

        // If no heartbeat for Return To Base amount of time
        if(hbeat_count > RTB_TIMEOUT)
        {
            string mode; n->getParam("/MODE", mode);
        
            // If we aren't already Returning to Base and competition mode engaged
            if (!rtb_engaged) 
            {
                if (mode == "COMPETITION")
                {
                  ROS_INFO("Heartbeat timeout reached, I'M COMING HOME FAM");
                  autopilot::calc_route srv;
                
                  srv.request.disable_lidar = true;
                  srv.request.glen_enabled = false; 

                  srv.request.latlng = true;
                  srv.request.destination = home_coord;
                
                  start_auto_client.call(srv); // Engage autonomous mode
                
                  rtb_engaged = true; // Record that we're engaging Return to Base
                }
                else Set_Mode_To_Standby();
            }	
        }

        // Check voltages
        if(voltages_count > VOLTAGES_TIMEOUT)
        {
            //Print_Voltage_Error();
            //Set_Mode_To_Standby();
        }

        // Check RPM
        if(rpm_count > RPM_TIMEOUT)
        {
            //Print_RPM_Error();
            //Set_Mode_To_Standby();
        }

        // Limit counte, 10x is sufficiently high
        if(hbeat_count < (10*RTB_TIMEOUT)) hbeat_count++;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
