#ifndef SERIAL_H_
#define SERIAL_H_
#include <stdint.h>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include "../../common/include/cppTypes.h"
#include "../../common/include/Dynamics/spatial.h"
#include "../../common/include/Math/orientation_tools.h"
#include "../../lcm-types/cpp/N100IMU_lcm.hpp"
#include<iostream>
#include<iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <mutex>

namespace N100IMU{

//N100惯导数据格式宏定义
#define IMU_LEN 0x38  //IMU数据长度
#define IMU_Acc_LEN 0x0c  //IMU_Acc数据长度
#define AHRS_LEN 0x30 //AHRS数据长度
#define FRAME_HEAD 0xFC //数据帧头
#define FRAME_END 0xFD //数据帧尾
#define TYPE_IMU 0x40 //IMU数据类别
#define TYPE_IMU_Acc 0x61 //IMU_Acc数据类别
#define TYPE_AHRS 0x41 //AHRS数据类别
#define IMU_TYPE_LEN 64 //当数据类型为IMU时，该数组的总长度
#define IMU_Acc_TYPE_LEN 24 //当数据类型为IMU_Acc时，该数组的总长度
#define AHRS_TYPE_LEN 56 //当数据类型为AHRS时，该数组的总长度


//创建IMU数据类型的类
class IMUPacket{
 public:
    float gyroscope_x;          //unit: rad/s
    float gyroscope_y;          //unit: rad/s
    float gyroscope_z;          //unit: rad/s
    float accelerometer_x;      //m/s^2
    float accelerometer_y;      //m/s^2
    float accelerometer_z;      //m/s^2
    float magnetometer_x;       //mG
    float magnetometer_y;       //mG
    float magnetometer_z;       //mG
    float imu_temperature;      //C
    float Pressure;             //Pa
    float pressure_temperature; //C
    uint32_t Timestamp;         //us
};

//创建IMU_Acc数据类型的类
class IMUAccPacket{
 public:
    float accelerometer_x;      //m/s^2
    float accelerometer_y;      //m/s^2
    float accelerometer_z;      //m/s^2
    uint32_t Timestamp;         //us
};

//创建AHRS数据类型的类
class AHRSPacket{
 public:
  float RollSpeed;   //unit: rad/s
  float PitchSpeed;  //unit: rad/s
  float HeadingSpeed;//unit: rad/s
  float Roll;        //unit: rad
  float Pitch;       //unit: rad
  float Heading;     //unit: rad
  float Qw;//w       //Quaternion
  float Qx;//x
  float Qy;//y
  float Qz;//z
  uint32_t Timestamp; //unit: us
};

//根据具体的设备修改

class Hardware{
  public:
    void Read_N100Data();
    void DataUnpacking();
    float HEX_to_Float_New(uint8_t *data,bool mode);
    long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);

    void updateLCM(LCM::N100IMU_lcm *message);
    bool initIMU();
    void run();

    uint32_t invalid_packets = 0;
    uint32_t timeout_packets = 0;
    uint32_t unknown_packets = 0;
    uint32_t good_packets = 0;

    //Data Parket for Cheetah Software 
    Vec3<float> gyro;
    Vec3<float> acc;
    Vec4<float> quat;

    const char* default_path = "/dev/ttyUSB0";
    int fd = -1;

};

}

#endif