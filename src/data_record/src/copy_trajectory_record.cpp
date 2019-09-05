#include "ros/ros.h"
#include "data_record/trajectory.h"
#include "data_record/trajectory_sim.h"
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
using namespace std;
void trajectory_simCallback(const data_record::trajectory_sim& msg)
{
    uint32_t dataNum;
    uint32_t len;
    char *BASEPATH  = "/home/zhaojt/ROS/";
    char *PATH;
    static ofstream location_out; 
    Trajectory Trajectory_data;
    if (strcmp(msg.cmd.c_str(),"start") == 0)
    {
        dataNum = 0;
        len = msg.dataNum;
        PATH = strcpy(BASEPATH,msg.dataName.c_str());
        location_out.open(PATH, std::ios::out | std::ios::app);
        if (!location_out.is_open())    
        {
            ROS_ERROR_STREAM("Unable to open file ");             
        }
    }
    else if (strcmp(msg.cmd.c_str(),"data") == 0)
    {
        dataNum = dataNum + 1;
        location_out << dataNum << ',' << msg.x << ',' << msg.y << ',0' << endl;
    }

    else if (strcmp(msg.cmd.c_str(),"stop") == 0)
    {
        location_out.close();
    }
    else {
    
    ROS_INFO("Cannot find legel command");

    }



}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory_record");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("trajectory_sim",1000,trajectory_simCallback); 
    ros::Rate loop_rate(1000);   
    while(ros::ok())
    {
        ros::spinOnce();    
        loop_rate.sleep(); 

    }

    return 0;
}



