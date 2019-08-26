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

#include "lds.h"

#include <stdio.h>
#include <time.h>
#include <chrono>

#include "delog.h"

/* Common function ------ ----------------------------------------------------------------------- */

bool IsFilePathValid(const char* path_str) {
  int str_len = strlen(path_str);

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)){
    return true;
  } else {
    return false;
  }
}

uint32_t GetPointInterval(uint32_t device_type) {
  if ((kDeviceTypeLidarTele == device_type) || \
      (kDeviceTypeLidarHorizon == device_type)) {
    return 4167; // 4167 ns,
  } else {
    return 10000; // ns
  }
}

uint32_t GetPacketNumPerSec(uint32_t device_type) {
  if ((kDeviceTypeLidarTele == device_type) || \
      (kDeviceTypeLidarHorizon == device_type)) {
    return 2400; // 2400 raw packet per second
  } else {
    return 1000; // 1000 raw packet per second
  }
}

uint32_t GetEthPacketLen(LivoxEthPacket* eth_packet, uint32_t point_num) {
  if (kCoordinateCartesian == eth_packet->data_type) {
    return (KEthPacketHeaderLength + point_num * KCartesianPointSize);
  } else {
    return (KEthPacketHeaderLength + point_num * KSphericalPointSzie);
  }
}

uint64_t GetStoragePacketTimestamp(StoragePacket* packet, uint8_t  data_src_) {

  LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(packet->raw_data);
  LdsStamp timestamp;
  memcpy(timestamp.stamp_bytes, raw_packet->timestamp, sizeof(timestamp));

  if (raw_packet->timestamp_type == kTimestampTypePps) {
    if (data_src_ != kSourceLvxFile) {
      return (timestamp.stamp + packet->time_rcv);
    } else {
      return timestamp.stamp;
    }
  } else if (raw_packet->timestamp_type == kTimestampTypeNoSync) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePtp) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year  = raw_packet->timestamp[0] + 100; // map 2000 to 1990
    time_utc.tm_mon   = raw_packet->timestamp[1];
    time_utc.tm_mday  = raw_packet->timestamp[2];
    time_utc.tm_hour  = raw_packet->timestamp[3];
    time_utc.tm_min   = 0;
    time_utc.tm_sec   = 0;

    uint64_t time_epoch = mktime(&time_utc);
    time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high; // to us
    time_epoch = time_epoch * 1000; // to ns

    return time_epoch;
  } else {
    printf("Timestamp type invalid.\n");
    return 0;
  }
}

uint32_t CalculatePacketQueueSize(uint32_t interval_ms, uint32_t device_type) {
  uint32_t queue_size = (interval_ms * GetPacketNumPerSec(device_type)) / 1000;
  queue_size = queue_size;
  if (queue_size < kMinEthPacketQueueSize) {
    queue_size = kMinEthPacketQueueSize;
  } else if (queue_size > kMaxEthPacketQueueSize) {
    queue_size = kMaxEthPacketQueueSize;
  }

  return queue_size;
}

void ParseCommandlineInputBdCode(const char* cammandline_str,
                                 std::vector<std::string>& bd_code_list) {
  char* strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char* bd_str  = strtok(strs, pattern.c_str());
  std::string invalid_bd = "000000000";
  while (bd_str != NULL) {
    printf("Commandline input bd:%s\n", bd_str);
    if ((kBdCodeSize == strlen(bd_str)) && \
        (NULL == strstr(bd_str, invalid_bd.c_str()))) {
      bd_code_list.push_back(bd_str);
    } else {
      printf("Invalid bd:%s!\n", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete [] strs;
}


#if 0

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point) {
  p_dpoint->x = p_raw_point->x/1000.0f;
  p_dpoint->y = p_raw_point->y/1000.0f;
  p_dpoint->z = p_raw_point->z/1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

#endif

/* Member function ------ ----------------------------------------------------------------------- */

Lds::Lds(uint32_t buffer_time_ms, uint8_t data_src)
    : buffer_time_ms_(buffer_time_ms), data_src_(data_src) {
  lidar_count_ = 0;
  request_exit_ = false;
  ResetLds(data_src_);
};

Lds::~Lds() {
  lidar_count_=0;
  ResetLds(0);
};

void Lds::ResetLidar(LidarDevice* lidar) {
  DeInitQueue(&lidar->data);

  memset(lidar, 0, sizeof(LidarDevice));

  /** unallocated state */
  lidar->handle = kMaxSourceLidar;
  lidar->connect_state = kConnectStateOff;
  lidar->data_src = 0;
}

void Lds::SetLidarDataSrc(LidarDevice* lidar, uint8_t data_src) {
  lidar->data_src = data_src;
}

void Lds::ResetLds(uint8_t data_src) {
  lidar_count_ = kMaxSourceLidar;
  for (uint32_t i=0; i<kMaxSourceLidar; i++) {
    Lds::ResetLidar(&lidars_[i]);
    Lds::SetLidarDataSrc(&lidars_[i], data_src);
  }
}

uint8_t Lds::GetDeviceType(uint8_t handle) {
  if (handle < kMaxSourceLidar) {
    return lidars_[handle].info.type;
  } else {
    return kDeviceTypeHub;
  }
}

void Lds::PrepareExit(void) {

}



