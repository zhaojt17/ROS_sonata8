#ifndef __GPS_IMU_PARSE_CORE_H_
#define __GPS_IMU_PARSE_CORE_H_

#include "GPS_parse.h"
#include "qxwz_rtcm.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <serial/serial.h>  
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include <stdint.h>


#define buadrate_gps 115200
#define buadrate_imu 115200
#define _ser_gps_front  "/dev/ttyS0"
#define _ser_gps_rear  "/dev/ttyS1"
#define _ser_imu "/dev/ttyS3"

#define latitude_orign 39.9997596270
#define longitude_orign 116.3302572240


class GpsImuCore
{
    public:

        GpsImuCore(ros::NodeHandle &nh);

    private:
    
        // gps and imu data
        Gps gps_front, gps_rear;
        Imu imu;
        Vehiclestate veh;
        Qxwz qxwz;
        
        //fuctions
        void open_serial(serial::Serial &ser,std::string &port,uint32_t baud);
        void read_gps_front(void);
        void read_gps_rear(void);
        void read_imu(void);
        void close_serial(void);
        void localize_gps_imu(void);


};
#endif
