//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// livox lidar data source

#ifndef COAT_LDS_H_
#define COAT_LDS_H_

#include <stdint.h>
#include <vector>
#include <string>

#include "ldq.h"

#include "livox_def.h"

/** Max lidar data source num */
const uint32_t kMaxSourceLidar = 32;

/** Eth packet relative info parama */
const uint32_t kMaxPointPerEthPacket        = 100;
const uint32_t kMinEthPacketQueueSize       = 32;          /**< must be 2^n */
const uint32_t kMaxEthPacketQueueSize       = 8192;        /**< must be 2^n */

const uint32_t KEthPacketHeaderLength       = 18;           /**< (sizeof(LivoxEthPacket) - 1) */
//const uint32_t KEthPacketMaxLength          = 1500;
const uint32_t KCartesianPointSize          = 13;
const uint32_t KSphericalPointSzie          = 9;

const uint64_t kPacketTimeGap               = 1000000;      /**< 1ms = 1000000ns */
const uint64_t kMaxPacketTimeGap            = 1700000;      /**< the threshold of packet continuous */
const uint64_t kDeviceDisconnectThreshold   = 1000000000;   /**< the threshold of device disconect */
const uint64_t kNsPerSecond                 = 1000000000;   /**< 1s  = 1000000000ns */

const int kPathStrMinSize                   = 4;            /**< Must more than 4 char */
const int kPathStrMaxSize                   = 256;          /**< Must less than 256 char */
const int kBdCodeSize                       = 15;


/** Lidar connect state */
typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateSampling = 2,
} LidarConnectState;

/** Device data source type */
typedef enum {
  kSourceRawLidar = 0,      /**< Data from raw lidar. */
  kSourceRawHub = 1,        /**< Data from lidar hub. */
  kSourceLvxFile,           /**< Data from parse lvx file. */
  kSourceUndef,
} LidarDataSourceType;

/** Lidar Data output type */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} LidarDataOutputType;

typedef enum {
  kCoordinateCartesian = 0,
  kCoordinateSpherical
} CoordinateType;

typedef struct {
  uint32_t receive_packet_count;
  uint32_t loss_packet_count;
  uint64_t last_timestamp;
  uint64_t timebase;            /**< unit:ns */
  uint32_t timebase_state;
} LidarPacketStatistic;

/** 8bytes stamp to uint64_t stamp */
typedef union {
  struct {
    uint32_t low;
    uint32_t high;
  } stamp_word;

  uint8_t stamp_bytes[8];
  uint64_t stamp;
} LdsStamp;

/** Lidar data source info abstract */
typedef struct {
  uint8_t handle;               /**< Lidar access handle. */
  uint8_t data_src;             /**< From raw lidar or livox file. */
  LidarConnectState connect_state;
  DeviceInfo info;
  LidarPacketStatistic statistic_info;
  LidarDataQueue data;
  uint32_t firmware_ver;        /**< Firmware version of lidar  */
} LidarDevice;

/**
 * Global function for general use.
 */
bool IsFilePathValid(const char* path_str);
uint32_t GetPointInterval(uint32_t device_type);
uint32_t GetPacketNumPerSec(uint32_t device_type);
uint32_t GetEthPacketLen(LivoxEthPacket* eth_packet, uint32_t point_num);
uint64_t GetStoragePacketTimestamp(StoragePacket* packet, uint8_t  data_src_);
uint32_t CalculatePacketQueueSize(uint32_t interval_ms, uint32_t device_type);
void ParseCommandlineInputBdCode(const char* cammandline_str,
                                   std::vector<std::string>& bd_code_list);

/**
 * Lidar data source abstract.
 */
class Lds {
public:
  Lds(uint32_t buffer_time_ms, uint8_t data_src);
  virtual ~Lds();

  uint8_t GetDeviceType(uint8_t handle);
  static void ResetLidar(LidarDevice* lidar);
  static void SetLidarDataSrc(LidarDevice* lidar, uint8_t data_src);
  void ResetLds(uint8_t data_src);
  void RequestExit() { request_exit_ = true; }
  void CleanRequestExit() { request_exit_ = false; }
  bool IsRequestExit() { return request_exit_; }
  virtual void PrepareExit(void);

  uint8_t lidar_count_;                 /**< Lidar access handle. */
  LidarDevice lidars_[kMaxSourceLidar]; /**< The index is the handle */

protected:
  uint32_t buffer_time_ms_;             /**< Buffer time before data in queue is read */
  uint8_t  data_src_;
private:
  volatile bool request_exit_;
};

#endif
