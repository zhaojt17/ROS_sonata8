#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
using namespace std;
static ofstream location_out; 
uint32_t dataNum = 0;
void trajectory_simCallback(const geometry_msgs::Point& msg)
{   
    uint32_t len = 1000;
    if(dataNum<len)
    {dataNum = dataNum + 1;
    location_out << dataNum << ',' << msg.x << ',' << msg.y;
    location_out << ',' << '0' << ',' << msg.z << endl;
    ROS_INFO("XIEXIENI");
    }
    if (dataNum == 1000)
    {
        location_out.close();
        dataNum = dataNum + 1;
    } 

}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory_record");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("trajectory_sim",1000,trajectory_simCallback); 
    ros::Rate loop_rate(1000);
    location_out.open("/home/zhaojt/ROS/test.csv", std::ios::out | std::ios::app);  
    if (!location_out.is_open())    
    {
        ROS_ERROR_STREAM("Unable to open file ");             
    } 
    while(ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep(); 

    }

    return 0;
}



