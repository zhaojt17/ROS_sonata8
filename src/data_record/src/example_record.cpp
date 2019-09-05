#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std;
int main(int argc,char **argv)
{   
    int count = 0;
    ros::init(argc,argv,"example_record");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);  
    ofstream location_out; 
    location_out.open("/home/zhaojt/ROS/test.txt", std::ios::out | std::ios::app);
    if (!location_out.is_open())    
    {
        ROS_ERROR_STREAM("Unable to open file "); 
        return 1;
    }
    while(ros::ok())
    {
        
        //ROS_INFO("transmit test, I transmit ID : %d",msg.frameId);
        ros::spinOnce();
        loop_rate.sleep();
        ++ count;
       
        /**************data record********/
        location_out << count << endl;

        /*******************************/
        if(count>=50)
        {
            location_out.close();
            return 0;
        }
    }

    return 0;

}