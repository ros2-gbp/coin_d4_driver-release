// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Hyeongjun Jeon

#ifndef COIN_D4_DRIVER__LIDAR_SDK__LIDAR_INFORMATION_HPP_
#define COIN_D4_DRIVER__LIDAR_SDK__LIDAR_INFORMATION_HPP_

#include <stdint.h>
#include <vector>

#define FRAME_ANGLE_RANGE (90 * 256)
#define FRAME_ANGLE_BASE (0xA000)

#define PI 3.141592654
#define ULTRASONIC_ANGLE_INC_DEG 0.5

constexpr uint8_t start_lidar[4] = {0xAA, 0x55, 0xF0, 0x0F};
constexpr uint8_t end_lidar[4] = {0xAA, 0x55, 0xF5, 0x0A};
constexpr uint8_t low_exposure[4] = {0xAA, 0x55, 0xF1, 0x0E};
constexpr uint8_t high_exposure[4] = {0xAA, 0x55, 0xF2, 0x0D};
constexpr uint8_t get_version[4] = {0xAA, 0x55, 0xFC, 0x03};

constexpr uint8_t high_speed[4] = {0xAA, 0x55, 0xF2, 0x0D};
constexpr uint8_t low_speed[4] = {0xAA, 0x55, 0xF1, 0x0E};

typedef struct
{
  float f_begin;
  float f_end;
} LidarCoverAngleStr;

typedef struct
{
  uint64_t timeStamp;
  float GyroYaw;
} GyroYawStr;

struct LaserConfig
{
  float min_angle;
  float max_angle;
  float angle_increment;
  float time_increment;
  float scan_time;
  float min_range;
  float max_range;
  LaserConfig & operator=(const LaserConfig & data)
  {
    min_angle = data.min_angle;
    max_angle = data.max_angle;
    angle_increment = data.angle_increment;
    time_increment = data.time_increment;
    scan_time = data.scan_time;
    min_range = data.min_range;
    max_range = data.max_range;
    return *this;
  }
};

struct node_info
{
  uint8_t sync_flag;
  uint16_t sync_quality;
  uint16_t angle_q6_checkbit;
  uint16_t distance_q2;
  uint64_t stamp;
  uint8_t scan_frequency;
  uint8_t exp_m;
  uint8_t debug_info[12];
  uint8_t index;
};

typedef enum
{
  DEFAULT_TIMEOUT = 2000,
  DEFAULT_HEART_BEAT = 1000,
  MAX_SCAN_NODES = 800,
  DEFAULT_TIMEOUT_COUNT = 10,
} TIME_CHECK;

typedef enum
{
  LIDAR_FOR_MOTION = 0,
  LIDAR_FOR_BLOCKED = 1,
} MOTION_AND_BLOCKED;

#define RESULT_OK 0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL -2

#define IS_OK(x) ((x) == RESULT_OK)
#define IS_TIMEOUT(x) ((x) == RESULT_TIMEOUT)
#define IS_FAIL(x) ((x) == RESULT_FAIL)

typedef int32_t result_t;

struct PackageNode
{
  uint8_t PakageSampleQuality;
  uint16_t PakageSampleDistance;
} __attribute__((packed));

#define PackageSampleMaxLngth 0x100
typedef enum
{
  CT_Normal = 0,
  CT_RingStart = 1,
  CT_Tail,
} CT;

typedef enum
{
  M1C1_Mini_v1 = 1,
  M1C1_Mini_v2,
  M1CT_Coin_Plus,
  M1CT_TOF,
} LIDAR_VERSION;


#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA
#define NORMAL_PACKAGE_SIZE 90
#define INTENSITY_NORMAL_PACKAGE_SIZE 130

struct node_package
{
  uint16_t package_Head;
  uint8_t package_CT;
  uint8_t nowPackageNum;
  uint16_t packageFirstSampleAngle;
  uint16_t packageLastSampleAngle;
  uint16_t checkSum;
  uint8_t packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));

/*对应M1C1_Mini v1版本*/
struct node_packages
{
  uint16_t package_Head;
  uint8_t package_CT;
  uint8_t nowPackageNum;
  uint16_t packageFirstSampleAngle;
  uint16_t packageLastSampleAngle;
  uint16_t checkSum;
  uint16_t packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));

/*对应M1CT_Coin_Plus 版本*/
struct node_package_coin
{
  uint8_t headL;
  uint8_t headH;
  uint8_t info;
  uint8_t sampleinfo;
  uint8_t speedL;
  uint8_t speedH;
  uint8_t startAngleL;
  uint8_t startAngleH;
  struct
  {
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
  } data[8];
  uint8_t stopAngleL;
  uint8_t stopAngleH;
  uint8_t csL;
  uint8_t csH;
} __attribute__((packed));


#define ANGLESTRLENMAX 32
#define LIDAR_CMD_SYNC_BYTE 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD 0x80
#define LIDAR_ANS_SYNC_BYTE1 0xA5
#define LIDAR_ANS_SYNC_BYTE2 0x5A
#define LIDAR_RESP_MEASUREMENT_CHECKBIT (0x1 << 0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT 8
#define LIDAR_RESP_MEASUREMENT_SYNCBIT (0x1 << 0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1
#define _countof(_Array) static_cast<int>(sizeof(_Array) / sizeof(_Array[0]))

struct node_Gyro
{
  float angle;
  int count;
  uint64_t time;
};


struct cmd_packet
{
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed));

struct lidar_ans_header
{
  uint8_t syncByte1;
  uint8_t syncByte2;
  uint32_t size : 30;
  uint32_t subType : 2;
  uint8_t type;
} __attribute__((packed));

struct LaserPoint
{
  float angle;
  float range;
  uint16_t intensity;
  int16_t range_check;
  LaserPoint & operator=(const LaserPoint & data)
  {
    this->angle = data.angle;
    this->range = data.range;
    this->intensity = data.intensity;
    this->range_check = data.range_check;
    return *this;
  }
};

struct LaserScan
{
  uint64_t stamp;
  std::vector<LaserPoint> points;
  LaserConfig config;
  LaserScan & operator=(const LaserScan & data)
  {
    this->points = data.points;
    this->stamp = data.stamp;
    this->config = data.config;
    return *this;
  }
};
#endif  // COIN_D4_DRIVER__LIDAR_SDK__LIDAR_INFORMATION_HPP_
