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

#include "lds_lvx.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>

#include "delog.h"
#include "lvx_file.h"

/** Const varible -------------------------------------------------------------------------------- */
const uint32_t kFrameBufferSize = 1024;

/** Gloable function in LdsLvx for callback */
void LdsLvx::ReadLvxFile(LdsLvx* lvx_this) {

  printf("Wait for reading lvx file.\n");
  while (!lvx_this->IsStarted());
  printf("Start to read lvx file.\n");

  int file_state = 0;
  int progress   = 0;
  while (true) {

    while(true) {
      file_state = lvx_this->lvx_file_->GetFrame(&lvx_this->lvx_frame);
      if (!file_state) {
        uint32_t package_count = lvx_this->lvx_frame.header.package_count;
        for (uint32_t i=0; i<package_count; i++) {
          LvxBasePackDetail* detail_packet = &(lvx_this->lvx_frame.packet[i]);
          LivoxEthPacket* eth_packet = (LivoxEthPacket*)(&detail_packet->version);
          int32_t handle = detail_packet->device_index;
          LidarDataQueue* p_queue = &lvx_this->lidars_[handle].data;

          while(QueueIsFull(p_queue)) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
          }

          QueuePushAny(p_queue, (uint8_t *)eth_packet, GetEthPacketLen(eth_packet, 100),\
                       0, 100);
        }
      } else {
        break;
      }

      if (progress != lvx_this->lvx_file_->GetLvxFileReadProgress()) {
        progress = lvx_this->lvx_file_->GetLvxFileReadProgress();
        printf("Read progress : %d \n", progress);
      }
    }

    if ((file_state == kLvxFileAtEnd) && LdsLvx::IsAllQueueEmpty(lvx_this)) {
      if (file_state != kLvxFileAtEnd) {
        printf("\nRead the lvx file error!\n");
      } else {
        printf("\nRead the lvx file complete!\n");
      }

      lvx_this->RequestExit();
      break;
    }
    //std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

uint32_t LdsLvx::FindMinRestSpace(LdsLvx* lvx_this) {
  uint32_t MinSpace = kFrameBufferSize;
  for (int i=0; i<lvx_this->lidar_count_; i++) {
    LidarDevice* p_lidar = &lvx_this->lidars_[i];
    if (MinSpace > QueueUnusedSize(&p_lidar->data)) {
      MinSpace = QueueUnusedSize(&p_lidar->data);
    }
  }

  return MinSpace;
}

bool LdsLvx::IsAllQueueEmpty(LdsLvx* lvx_this) {
  for (int i=0; i<lvx_this->lidar_count_; i++) {
    LidarDevice* p_lidar = &lvx_this->lidars_[i];
    if (!QueueIsEmpty(&p_lidar->data)) {
      return false;
    }
  }

  return true;
}

/** For device connect use ---------------------------------------------------------------------- */
LdsLvx::LdsLvx(uint32_t interval_ms) : Lds(interval_ms, kSourceLvxFile) {
  start_read_lvx_ = false;
  is_initialized_ = false;
  lvx_file_ = std::make_shared<LvxFileHandle>();
  lvx_frame.packet = new LvxBasePackDetail[kFrameBufferSize];

  t_read_lvx_ = std::make_shared<std::thread>(std::bind(LdsLvx::ReadLvxFile,this));
  t_read_lvx_->detach();
}

LdsLvx::~LdsLvx() {
  if (lvx_frame.packet != nullptr) {
    delete [] lvx_frame.packet;
  }
}

void LdsLvx::PrepareExit(void) {
  lvx_file_->CloseLvxFile();
  printf("Lvx to rosbag convert complete and exit!\n");
}


int LdsLvx::InitLdsLvx(const char* lvx_path) {
  int ret = lvx_file_->Open(lvx_path, std::ios::in);
  if (ret) {
    printf("Open %s file fail!\n", lvx_path);
    return ret;
  }

  LdsLvx::ResetLds(kSourceLvxFile);

  lidar_count_ = lvx_file_->GetDeviceCount();
  if (!lidar_count_ || (lidar_count_ >= kMaxSourceLidar)) {
    lvx_file_->CloseLvxFile();
    printf("Lidar count error in %s : %d\n", lvx_path, lidar_count_);
    return -1;
  }
  printf("LvxFile[%s] have %d lidars\n", lvx_path, lidar_count_);

  for (int i=0; i<lvx_file_->GetDeviceCount(); i++) {
    LvxDeviceInfo lvx_dev_info;
    lvx_file_->GetDeviceInfo(i, &lvx_dev_info);
    lidars_[i].handle = i;
    lidars_[i].connect_state = kConnectStateSampling;
    lidars_[i].info.handle = i;
    lidars_[i].info.type = lvx_dev_info.device_type;
    memcpy(lidars_[i].info.broadcast_code, lvx_dev_info.lidar_broadcast_code,\
           sizeof(lvx_dev_info.lidar_broadcast_code));
    lidars_[i].data_src = kSourceLvxFile;

    uint32_t queue_size = kMaxEthPacketQueueSize * 16;
    InitQueue(&lidars_[i].data, queue_size);
    printf("Device[%s] queue size : %d\n", lvx_dev_info.lidar_broadcast_code, queue_size);
  }

  is_initialized_ = true;

  LdsLvx::StartRead();

  return ret;
}


