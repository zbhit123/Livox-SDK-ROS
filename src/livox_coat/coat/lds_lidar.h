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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef COAT_LDS_LIDAR_H_
#define COAT_LDS_LIDAR_H_

#include <memory>
#include <vector>

#include "lds.h"
#include "livox_def.h"
#include "livox_sdk.h"

typedef struct {
  uint64_t total_count;
  uint64_t period_ns;
  uint32_t packet_period_ns;
  uint8_t ref_lidar;
  uint8_t state;
} PpsEstimate;


/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar : public Lds {
public:
  static LdsLidar* GetInstance(uint32_t interval_ms) {
    static LdsLidar lds_lidar(interval_ms);
    return &lds_lidar;
  }

  int InitLdsLidar(std::vector<std::string>& broadcast_code_strs);
  int DeInitLdsLidar(void);

 private:
  LdsLidar(uint32_t interval_ms);
  LdsLidar(const LdsLidar&) = delete;
  ~LdsLidar();
  LdsLidar& operator=(const LdsLidar&) = delete;
  virtual void PrepareExit(void);

  static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,\
                             uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(uint8_t status, uint8_t handle, uint8_t response, void *clent_data);
  static void StopSampleCb(uint8_t status, uint8_t handle, uint8_t response, void *clent_data);
  static void DeviceInformationCb(uint8_t status, uint8_t handle, \
                                  DeviceInformationResponse *ack, void *clent_data);

  void ResetLdsLidar(void);
  int AddBroadcastCodeToWhitelist(const char* broadcast_code);
  void AddLocalBroadcastCode(void);
  bool FindInWhitelist(const char* broadcast_code);

  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

};

#endif
