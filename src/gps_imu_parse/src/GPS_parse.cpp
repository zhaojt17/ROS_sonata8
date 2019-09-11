#include "gps_imu_parse/GPS_parse.h"
#include "gps_imu_parse/gps_imu_parse_core.h"
#include <iostream>

extern serial::Serial ser_gps_front; 
extern serial::Serial ser_gps_rear; 
extern serial::Serial ser_imu; 

using namespace std;

void Gps::Gps_update(void)
{
    uint16_t i = 0;
	int16_t front = 0,tail = 0;

    while (i < rawdata_length)
    {   

        if ((rawdata[i] == 0xb5) && (rawdata[i+1] == 0x62))
        {

            front = i;
            tail = i + 7 + rawdata[i+4] + rawdata[i+5]*256;
            if (tail > rawdata_length) break;
            Ubx_parse(i, tail - front + 1);
        } 
        
        i++;
    }
}

uint8_t Gps::Ubx_parse(uint16_t head, uint16_t length)
{
    uint8_t ubx_class, ubx_id;
    uint16_t ubx_ck_a = 0, ubx_ck_b =0, payload_length;
    uint16_t i; 
    
    ubx_class = rawdata[head + 2];
    ubx_id = rawdata[head + 3];
    payload_length = rawdata[head + 4] + rawdata[head + 5] * 256;

    if (ubx_class != UBX_CLASS_NAV) return 0;

    // check sum 
    for(i = 0; i < payload_length + 4; i++)
	{
		ubx_ck_a += rawdata[head + 2 + i];
		ubx_ck_b += ubx_ck_a ;
	}
    if (((ubx_ck_a & 0xff) != rawdata[head + length - 2]) || ((ubx_ck_b & 0xff) != rawdata[head + length - 1])) return 0;
    
    //load the information 

    switch (ubx_id)
    {
        case UBX_HIGHPRELLA:

            // longitude;

            decoder.u8_val[0] = rawdata[head + 14];
            decoder.u8_val[1] = rawdata[head + 15];
            decoder.u8_val[2] = rawdata[head + 16];
            decoder.u8_val[3] = rawdata[head + 17];
            longitude = decoder.i32_val / 10000000.0 + rawdata[head +30] / 1000000000.0;
            
            // latitude;
            
            decoder.u8_val[0] = rawdata[head + 18];
            decoder.u8_val[1] = rawdata[head + 19];
            decoder.u8_val[2] = rawdata[head + 20];
            decoder.u8_val[3] = rawdata[head + 21];
            latitude = decoder.i32_val / 10000000.0 + rawdata[head +31] / 1000000000.0;
            
            // height_wgs84;
            decoder.u8_val[0] = rawdata[head + 22];
            decoder.u8_val[1] = rawdata[head + 23];
            decoder.u8_val[2] = rawdata[head + 24];
            decoder.u8_val[3] = rawdata[head + 25];
            height_wgs84 = (decoder.i32_val + rawdata[head +32]) / 1000.0;
            
            break;

        case UBX_NAV_PVT:
            
            // itow (s)
            decoder.u8_val[0] = rawdata[head + 6];
            decoder.u8_val[1] = rawdata[head + 7];
            decoder.u8_val[2] = rawdata[head + 8];
            decoder.u8_val[3] = rawdata[head + 9];
            itow = decoder.u32_val / 1000.0; 

            // year; month; day; hour; min; sec;
            year = rawdata[head + 10] + rawdata[head + 11] * 256;
            month = rawdata[head + 12];
            day = rawdata[head + 13];
            hour = rawdata[head + 14];
            min = rawdata[head + 15];
            sec = rawdata[head + 16];

            // fixmode; numsv;
            fixmode = rawdata[head + 27];
            num_SV = rawdata[head + 29];

            // pos std (m)
            decoder.u8_val[0] = rawdata[head + 46];
            decoder.u8_val[1] = rawdata[head + 47];
            decoder.u8_val[2] = rawdata[head + 48];
            decoder.u8_val[3] = rawdata[head + 49];
            horizon_pos_std = decoder.u32_val / 100.0;

            decoder.u8_val[0] = rawdata[head + 50];
            decoder.u8_val[1] = rawdata[head + 51];
            decoder.u8_val[2] = rawdata[head + 52];
            decoder.u8_val[3] = rawdata[head + 53];
            vertical_pos_std = decoder.u32_val / 100.0;
            
            // vel (m/s)
            decoder.u8_val[0] = rawdata[head + 54];
            decoder.u8_val[1] = rawdata[head + 55];
            decoder.u8_val[2] = rawdata[head + 56];
            decoder.u8_val[3] = rawdata[head + 57];
            vel_N = decoder.i32_val / 1000.0;

            decoder.u8_val[0] = rawdata[head + 58];
            decoder.u8_val[1] = rawdata[head + 59];
            decoder.u8_val[2] = rawdata[head + 60];
            decoder.u8_val[3] = rawdata[head + 61];
            vel_E = decoder.i32_val / 1000.0;

            decoder.u8_val[0] = rawdata[head + 62];
            decoder.u8_val[1] = rawdata[head + 63];
            decoder.u8_val[2] = rawdata[head + 64];
            decoder.u8_val[3] = rawdata[head + 65];
            vel_D = decoder.i32_val / 1000.0;

            decoder.u8_val[0] = rawdata[head + 66];
            decoder.u8_val[1] = rawdata[head + 67];
            decoder.u8_val[2] = rawdata[head + 68];
            decoder.u8_val[3] = rawdata[head + 69];
            ground_speed = decoder.i32_val / 1000.0;

            decoder.u8_val[0] = rawdata[head + 74];
            decoder.u8_val[1] = rawdata[head + 75];
            decoder.u8_val[2] = rawdata[head + 76];
            decoder.u8_val[3] = rawdata[head + 77];
            vel_std = decoder.u32_val / 1000.0;

            //  heading angle
            decoder.u8_val[0] = rawdata[head + 70];
            decoder.u8_val[1] = rawdata[head + 71];
            decoder.u8_val[2] = rawdata[head + 72];
            decoder.u8_val[3] = rawdata[head + 73];
            head_motion = decoder.i32_val / 100000.0;
            
            decoder.u8_val[0] = rawdata[head + 78];
            decoder.u8_val[1] = rawdata[head + 79];
            decoder.u8_val[2] = rawdata[head + 80];
            decoder.u8_val[3] = rawdata[head + 81];
            head_std = decoder.u32_val / 100000.0;

            break;
    }

    return 1;
}

// qxwz rtcm service 


void qxwz_rtcm_response_callback(qxwz_rtcm data)
{   
    uint8_t buffer[1024];
    uint16_t i;

    for (i = 0; i < data.length; i++)
    {
        buffer[i] = data.buffer[i];
    }

    if (ser_gps_front.isOpen())
    {
        ser_gps_front.write(buffer, data.length);
    } 
    
    if (ser_gps_rear.isOpen())
    {
        ser_gps_rear.write(buffer, data.length);
    }  

   // ROS_INFO("receive rtcm3");
}

void qxwz_status_response_callback(qxwz_rtcm_status code)
{
  	ROS_INFO("status_code = %d",code);
}



void Qxwz::rtcm_initial(void)
{
    //设置appKey和appSecret
    //apapKey申请详细见说明文档
    config.appkey="660182";
    config.appSecret="6193127c33ac1c99a6b617c608e42b2b2689b597b86ba2308b8f73909a68d8a7";
    config.deviceId="VDC_QXWZ01";
    config.deviceType="TSET";

    //[1] Set sdk configs
    qxwz_setting(&config);

    //[2] Start rtcm sdk
    qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);
    ROS_INFO("qxwz service started");
}      

void Qxwz::update_gpgga(void)
{
    uint16_t i, j;
    char GGAMSG[1024];

    if (rawdata_length > 1024) return;

    for (i = 0; i < (rawdata_length - 4); i++)
    {   
       
        if ((rawdata[i] == '$') && (rawdata[i+1] == 'G') && (rawdata[i+3] == 'G') && (rawdata[i+4] == 'G') && (rawdata[i+5] == 'A'))
        {
            
            for(j = i; j < (rawdata_length - 1); j++)
            {
                GGAMSG[j-i] = rawdata[j];
                   
            }
            
            //GGAMSG[j-i+1] = '\r';
            //GGAMSG[j-i+2] = '\n';
            GGAMSG[j-i+3] = '\0';
            //GGAMSG[1] = 'P';
            //ROS_INFO("%s", GGAMSG);
            //qxwz_rtcm_sendGGAWithGGAString(GGAMSG); 
            qxwz_rtcm_sendGGAWithGGAString("$GNGGA,104441.60,3959.95061,N,11619.77259,E,5,12,0.62,51.9,M,-8.7,M,24.6,0446*79\r\n"); 
            //qxwz_rtcm_sendGGAWithGGAString("$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B\r\n");
            
            //ROS_INFO("send GGAMSG success");

            return;
        } 
    }

}