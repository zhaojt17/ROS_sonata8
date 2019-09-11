#ifndef __GPS_PARSE_H_
#define __GPS_PARSE_H_

#include <stdint.h>
#include "qxwz_rtcm.h"

#define UBX_HIGHPRELLA 0x14
#define UBX_NAV_PVT 0x07
#define UBX_CLASS_NAV 0x01

typedef union 
{
    float f_val;
    uint8_t u8_val[4];
    uint32_t u32_val;
    int32_t i32_val;
} decoder_4_bytes;


class Gps
{
    public:

        //--------------variables----------------//
        //time 
        float itow;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;

        // position 
        double latitude;
        double longitude;
        double height_wgs84;
        double vertical_pos_std;
        double horizon_pos_std;

        // velocity
        double vel_N;
        double vel_E;
        double vel_D;
        double ground_speed;
        double vel_std;
 
        //  heading angle
        double head_motion;
        double head_std;

        // fixmod
        uint8_t fixmode;
        uint8_t num_SV;
        
        // raw data
        uint8_t rawdata[1024];
        uint16_t rawdata_length;
        
        // datadecoder
        decoder_4_bytes decoder;

        //--------------fuctions----------------//
        uint8_t Ubx_parse(uint16_t head, uint16_t length);
        void Gps_update(void);
        void Read_from_serial(void);
};      

class Imu
{
    public:
    
        double timestamp;
        double acc_x;
        double acc_y;
        double acc_z;
        double gyr_x;
        double gyr_y;
        double gyr_z;
};

class Vehiclestate
{
    public:

        double timestamp;
        double pos_x;
        double pos_y;
        double pos_z;
        double phi_z;
        double vel_x;
        double vel_y;
        double vel_z;
        double acc_x;
        double acc_y;
        double acc_z;
        double speed;
};

class Qxwz
{
    public:

        qxwz_config config;
       
        uint8_t rawdata[1024];
        uint16_t rawdata_length;
       
        void rtcm_initial(void);
        void update_gpgga(void);

    private:
};

#endif