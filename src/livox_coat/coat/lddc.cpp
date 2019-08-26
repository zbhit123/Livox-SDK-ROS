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

#include "lddc.h"

#include <stdint.h>
#include <inttypes.h>


#include "delog.h"
#include "lds_lvx.h"
#include "lds_lidar.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <livox_coat/CustomPoint.h>
#include <livox_coat/CustomMsg.h>

const uint32_t kPublishPacketNumLimit       = 10; /* Max packets for one publish  */


Lddc::Lddc(int format, int multi_topic, int data_src, int output_type, double frq) : \
    transfer_format_(format), use_multi_topic_(multi_topic),\
    data_src_(data_src), output_type_(output_type), publish_frq_(frq) {

  publish_interval_ms_ = 1000/publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  global_pub_ = nullptr;
  cur_node_   = nullptr;
  bag_        = nullptr;
};

Lddc::~Lddc() {

  if (global_pub_) {
    delete global_pub_;
  }

  if (lds_) {
    delete lds_;
  }

  for (uint32_t i=0; i<kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }
}

uint32_t Lddc::PublishPointcloud2(LidarDataQueue* queue, uint32_t packet_num, \
                                   uint8_t handle, uint8_t data_source) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;
  uint32_t point_num = 0;
  uint32_t kPointXYZISize = 16;
  sensor_msgs::PointCloud2 cloud;

  cloud.header.frame_id = "livox_frame";
  cloud.height = 1;
  cloud.width  = 0;

  cloud.fields.resize(4);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name   = "x";
  cloud.fields[0].count  = 1;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name   = "y";
  cloud.fields[1].count  = 1;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name   = "z";
  cloud.fields[2].count  = 1;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name   = "intensity";
  cloud.fields[3].count  = 1;
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

  cloud.data.resize(packet_num * kMaxPointPerEthPacket * kPointXYZISize);
  cloud.point_step    = kPointXYZISize;
  uint8_t *point_base = cloud.data.data();
  StoragePacket storage_packet;
  while (published_packet <  packet_num) {
    QueueProPop(queue, &storage_packet);
    LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    LivoxRawPoint*  raw_points = reinterpret_cast<LivoxRawPoint *>(raw_packet->data);

    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    if (published_packet && \
       ((timestamp - last_timestamp) > kMaxPacketTimeGap)) {
      if (kSourceLvxFile != data_source) {
        break;
      }
    }

    if (!cloud.width) {
      cloud.header.stamp = ros::Time(timestamp/1000000000.0); // to ros time stamp
    }
    cloud.width += storage_packet.point_num;

    for (uint32_t i = 0; i < storage_packet.point_num; i++) {
      if (kSourceLvxFile == data_source) {
        LivoxPoint* livox_points = reinterpret_cast<LivoxPoint *>(raw_points);
        *((float*)(point_base +  0)) = livox_points->x;
        *((float*)(point_base +  4)) = livox_points->y;
        *((float*)(point_base +  8)) = livox_points->z;
      } else {
        *((float*)(point_base +  0)) = raw_points->x/1000.0f;
        *((float*)(point_base +  4)) = raw_points->y/1000.0f;
        *((float*)(point_base +  8)) = raw_points->z/1000.0f;
      }

      *((float*)(point_base + 12)) = (float) raw_points->reflectivity;
      ++raw_points;
      ++point_num;
      point_base += kPointXYZISize;
    }

    QueuePopUpdate(queue);
    last_timestamp = timestamp;
    ++published_packet;
  }

  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense     = true;
  // adjust the real size
  cloud.data.resize(cloud.row_step);

  ros::Publisher* p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(cloud);
  } else {
    if (bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp/1000000000.0), cloud);
    }
  }

  return published_packet;
}

uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue* queue, uint32_t packet_num,\
                                      uint8_t handle, uint8_t data_source) {
  static uint32_t msg_seq = 0;
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;
  uint32_t point_interval = GetPointInterval(lds_->GetDeviceType(handle));
  uint32_t packet_offset_time = 0; // ns

  livox_coat::CustomMsg livox_msg;

  livox_msg.header.frame_id = "livox_frame";
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
  livox_msg.header.stamp = ros::Time::now();
  livox_msg.timebase = 0;
  livox_msg.point_num = 0;
  livox_msg.lidar_id  = handle;

  StoragePacket storage_packet;
  while (published_packet <  packet_num) {
    QueueProPop(queue, &storage_packet);
    LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    LivoxRawPoint* raw_points = reinterpret_cast<LivoxRawPoint *>(raw_packet->data);

    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    if (published_packet && \
        ((timestamp - last_timestamp) > kDeviceDisconnectThreshold)) {
      ROS_INFO("Lidar[%d] packet loss", handle);
      break;
    }
    if (!livox_msg.timebase) {
      livox_msg.timebase = timestamp; // to us
      packet_offset_time = 0;         // first packet
      //ROS_DEBUG("[%d]:%ld %d", handle, livox_msg.timebase, point_interval);
    } else {
      packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
    }
    livox_msg.point_num += storage_packet.point_num;

    for (uint32_t i = 0; i < storage_packet.point_num; i++) {
      livox_coat::CustomPoint point;
      point.offset_time = packet_offset_time + i*point_interval;
      point.x = raw_points->x/1000.0f;
      point.y = raw_points->y/1000.0f;
      point.z = raw_points->z/1000.0f;
      point.reflectivity = raw_points->reflectivity;
      point.line = 0;
      ++raw_points;
      livox_msg.points.push_back(point);
    }

    QueuePopUpdate(queue);
    last_timestamp = timestamp;
    ++published_packet;
  }

  ros::Publisher* p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(livox_msg);
  } else {
    if (bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time::now(), livox_msg);
    }
  }

  return published_packet;
}

int Lddc::RegisterLds(Lds* lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributeLidarData(void) {
  if (lds_ == nullptr) {
    return;
  }

  for (int i = 0; i < lds_->lidar_count_; i++) {
    LidarDevice* lidar = &lds_->lidars_[i];
    LidarDataQueue* p_queue  = &lidar->data;
    if (kConnectStateSampling!= lidar->connect_state) {
      continue;
    }

    while (!QueueIsEmpty(p_queue)) {
      //ROS_DEBUG("%d %d %d %d\r\n", i, p_queue->rd_idx, p_queue->wr_idx, QueueUsedSize(p_queue));
      uint32_t used_size = QueueUsedSize(p_queue);
      if (used_size > kPublishPacketNumLimit) {
        used_size = kPublishPacketNumLimit;
      }

      uint8_t data_src = lidar->data_src;
      if (kPointCloud2Msg == transfer_format_) {
        if (used_size == PublishPointcloud2(p_queue, used_size, i, data_src)) {
        }
      } else {
        if (used_size == PublishCustomPointcloud(p_queue, used_size, i, data_src)) {
        }
      }
    }
  }

  if (lds_->IsRequestExit()) {
    Lddc::PrepareExit();
  }
}

ros::Publisher* Lddc::GetCurrentPublisher(uint8_t handle) {
  ros::Publisher** pub = nullptr;

  if (use_multi_topic_) {
    pub = &private_pub_[handle];
  } else {
    pub = &global_pub_;
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      ROS_INFO("Support multi topics.");
      snprintf(name_str, sizeof(name_str), "livox/lidar%d", handle);
    } else {
      ROS_INFO("Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub = cur_node_->advertise<sensor_msgs::PointCloud2>(name_str,\
                                                       kMinEthPacketQueueSize);
      ROS_INFO("Publish Use PointCloud2 Format!\n");
    } else {
      **pub = cur_node_->advertise<livox_coat::CustomMsg>(name_str,\
                                                       kMinEthPacketQueueSize);
      ROS_INFO("Publish Use Livox Custom Format!\n");
    }
  }

  return *pub;
}

void Lddc::CreateBagFile(const std::string& file_name) {
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    ROS_INFO("Create bag file :%s!\n", file_name.c_str());
  }
}

void Lddc::PrepareExit(void) {
  if (bag_) {
    bag_->close();
    lds_->PrepareExit();

    ROS_INFO("Press [Ctrl+C] to exit!\n");
    bag_ = nullptr;
  }
}


