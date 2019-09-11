#include "gps_imu_parse/gps_imu_parse_core.h"

#include <iostream>


std::string serport_gps_front = _ser_gps_front;
std::string serport_gps_rear = _ser_gps_rear;
std::string serport_imu = _ser_imu;

// hardware serial port
serial::Serial ser_gps_front; 
serial::Serial ser_gps_rear; 
serial::Serial ser_imu; 

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_imu");

    ros::NodeHandle nh;

    GpsImuCore core(nh);
    
   
    return 0;
}

GpsImuCore::GpsImuCore(ros::NodeHandle &nh)
{
    

    
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("trajectory_gps",1000);
    ros::Publisher pospub = nh.advertise<geometry_msgs::Pose2D>("position",1000);
    // open serial ports
    open_serial(ser_gps_front, serport_gps_front, buadrate_gps);
    open_serial(ser_gps_rear, serport_gps_rear, buadrate_gps);
    //open_serial(ser_imu, serport_imu,buadrate_imu);


    qxwz.rtcm_initial();

    ros::Rate loop_rate(10);

    geometry_msgs::Point gps_msg;
    geometry_msgs::Pose2D pos_msg;

    ser_gps_front.flush();
    ser_gps_rear.flush();

    uint8_t count = 0;
    while(ros::ok())
    {    
        read_gps_front();
        read_gps_rear();
        //read_imu();
        localize_gps_imu();
        qxwz.update_gpgga();


        gps_msg.x = veh.pos_x;
        gps_msg.y = veh.pos_y;
        gps_msg.z = veh.speed;    

        pos_msg.x = gps_rear.longitude;
        pos_msg.y = gps_rear.latitude;
        pos_msg.theta = veh.phi_z;
        

        pub.publish(gps_msg);
        if (count == 5)
        {
            pospub.publish(pos_msg);
            count =0;
        }
        
        count ++;
        
        loop_rate.sleep();
        ros::spinOnce();
    }

    close_serial();


    
    
}

void GpsImuCore::open_serial(serial::Serial &ser, std::string &port, uint32_t baud)
{
    try{
    ser.setPort(port); 
    ser.setBaudrate(baud); 
    serial::Timeout to= serial::Timeout::simpleTimeout(1000); 
    ser.setTimeout(to); 
    ser.open();
    }

    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open " + port);         
    } 

    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM(port + "initialized"); 
    } 
}


void GpsImuCore::read_gps_front(void)
{
    uint16_t buff_max_len = 1024, read_len = 0, i;
    uint8_t buff[buff_max_len];
    uint8_t flag;


    flag = 0;
    if (ser_gps_front.isOpen())
        { 
          
            if (ser_gps_front.available() > 0) 
            {   
                gps_front.rawdata_length = ser_gps_front.read(gps_front.rawdata, ser_gps_front.available());   
                qxwz.rawdata_length = gps_front.rawdata_length;

                for (i = 0; i < qxwz.rawdata_length; i++)
                {
                    qxwz.rawdata[i] = gps_front.rawdata[i];
         
                }    
 
                gps_front.Gps_update();
            }  
        }
   

    // cout << dec << (int)gps_front.year << endl;
    // cout << dec << (int)gps_front.min << endl;
    // cout << dec << (int)gps_front.sec << endl;
    // cout << "start"<<endl;
    
    // cout << fixed << setprecision(10) << gps_front.latitude << endl;
    // cout << fixed << setprecision(10) << gps_front.longitude << endl;
    // cout << "xiexieni" << endl;
    // cout << fixed << setprecision(10) << gps_rear.latitude << endl;
    // cout << fixed << setprecision(10) << gps_rear.longitude << endl;
    
}

void GpsImuCore::read_gps_rear(void)
{
    if (ser_gps_rear.isOpen())
        {  
            if (ser_gps_rear.available()) 
            {
                gps_rear.rawdata_length = ser_gps_rear.read(gps_rear.rawdata, ser_gps_rear.available()); 
                gps_rear.Gps_update();               
            }  
        }
    
}

void GpsImuCore::read_imu(void)
{
    ;
}

void GpsImuCore::close_serial(void)
{
    ser_gps_front.close();
}
/// locate the vehicle with gps and imu
void GpsImuCore::localize_gps_imu(void)
{
    static double x,y;
    static double temp;
    veh.timestamp = gps_rear.itow;
    veh.pos_x = (gps_rear.longitude - longitude_orign) * 5105840.41295 *0.01745329252;
    veh.pos_y = (gps_rear.latitude - latitude_orign) * 6378137 *0.01745329252;
    veh.speed = gps_rear.ground_speed;


    if (abs(gps_rear.itow - gps_front.itow) <= 0.2)
    {
        x = (gps_front.longitude -  gps_rear.longitude) * 5105840.41295 *0.01745329252;
	    y = (gps_front.latitude - gps_rear.latitude) * 6378137 *0.01745329252;
        
       
        if ((x==0)&&(y==0))
        {
            veh.phi_z = 100;
        }
        else
        {
            veh.phi_z = atan2(y, x);
            if(veh.phi_z > 6.2831853071795864){veh.phi_z -= 6.2831853071795864;}
	        if(veh.phi_z < 0){veh.phi_z += 6.2831853071795864;}
        }
        
    }
    

}



