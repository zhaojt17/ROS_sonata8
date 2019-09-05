#include "drivers/example_gnss.h"


serial::Serial ser; //声明串口对象 


int main(int argc, char** argv)
{   
    
    
    unsigned char buf[3];
    buf[0]='a';
    buf[1]='b';
    buf[2]='c';
    buf[3]='d';                    //定义字符串长度
    ros::init(argc,argv,"example_gnss");
    ros::NodeHandle n;
    
    /*****open device*******/
    try 
    { 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    ros::Rate loop_rate(100);


    while(ros::ok())
    {       
            //ser.write(buf,4);
        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << result.data); 
        } 

        ros::spinOnce();
        loop_rate.sleep();
    }

}

