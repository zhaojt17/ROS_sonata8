#include "controller/example_controller.h"


void can_receiveCallback(const can_driver::can_receive&  msg)
{
    int i=0;
    uint8_t data[8]; 
    for (i=0;i<8;i++) data[i] = msg.data[i];
    ROS_INFO("I heard ID : %d, data is : %u %u %u %u %u %u %u %u",msg.frameId,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
}


int main(int argc, char** argv)
{
    int i=0;
    example_controller new_example_controller;
    uint8_t count = 0;
    ros::init(argc,argv,"example_controller");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<can_driver::can_transmit>("can_transmit",1000);
    ros::Subscriber sub = n.subscribe("can_receive",1000,can_receiveCallback);
    ros::Rate loop_rate(1);
    can_driver::can_transmit msg;
    uint8_t data[8];
    while(ros::ok())
    {   
        {///////controller calculate //////////
        


        }

        {///////// pulish data /////////////
            for (i=0;i<8;i++) msg.data[i] = data[i];
            msg.frameId =0x31A;
            chatter_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++ count;
    }

}




