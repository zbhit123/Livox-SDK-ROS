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

#include <time.h>
#include <string.h>
#include <cmath>
#include "lvx_file.h"
#include "delog.h"

#define WRITE_BUFFER_LEN    1024 * 1024
#define MAGIC_CODE          (0xac0ea767)
#define PACK_POINT_NUM      100
#define M_PI                3.14159265358979323846
#define LVX_HEADER_SIG_STR  "livox_tech"

LvxFileHandle::LvxFileHandle() : device_count_(0), cur_frame_index_(0), cur_offset_(0),\
  data_start_offset_(0), size_(0) {
  state_ = kLvxFileUndefFault;
}

bool LvxFileHandle::ReadAndCheckHeader() {
  const char* sig_str = LVX_HEADER_SIG_STR;
  lvx_file_.seekg(0, std::ios::beg);
  lvx_file_.read((char *)(&header_), sizeof(header_));

  if (strcmp((const char *)header_.signature, sig_str)) {
    return false;
  }

  return true;
}


bool LvxFileHandle::AddAndCheckDeviceInfo() {
  lvx_file_.seekg (LvxFileHandle::DeviceInfoOffset(), std::ios::beg);
  lvx_file_.read((char *)(&device_count_), sizeof(device_count_));

  if (!device_count_) {
    return false;
  }

  for (int i=0; i< device_count_; i++) {
    LvxDeviceInfo device_info;
    lvx_file_.read((char *)(&device_info), sizeof(LvxDeviceInfo));
    AddDeviceInfo(device_info);
  }

  return true;
}

int LvxFileHandle::GetDeviceInfo(uint8_t idx, LvxDeviceInfo* info) {
  if (idx < device_info_list_.size()) {
    *info = device_info_list_[idx];

    return 0;
  }

  return -1;
}


bool LvxFileHandle::PrepareDataRead() {
  lvx_file_.seekg (LvxFileHandle::DataStartOffset(), std::ios::beg);

  FrameHeader frame_header;
  lvx_file_.read((char *)(&frame_header), sizeof(frame_header));

  if ((frame_header.current_offset != LvxFileHandle::DataStartOffset()) ||\
      (frame_header.frame_index != 0)) {
    return false;
  }

  // reset the read position to the start offset of data erea
  lvx_file_.seekg (LvxFileHandle::DataStartOffset(), std::ios::beg);

  return true;
}

int LvxFileHandle::Open(const char* filename,std::ios_base::openmode mode) {

  if ((mode & std::ios::in) == std::ios::in) {
    state_ = kLvxFileOk;
    lvx_file_.open(filename, mode | std::ios_base::binary | std::ios_base::ate);

    if (!lvx_file_.is_open()) {
      state_ = kLvxFileNotExist;
      return state_;
    }

    size_ = lvx_file_.tellg();
    lvx_file_.seekg (0, std::ios::beg);

    if (size_ < LvxFileHandle::MiniFileSize()) {
      state_ = kLvxFileSizeFault;

      return state_;
    }

    if (!LvxFileHandle::ReadAndCheckHeader()) {
      state_ = kLvxFileHeaderFault;
      return state_;
    }

    if (!LvxFileHandle::AddAndCheckDeviceInfo()) {
      state_ = kLvxFileDeviceInfoFault;
      return state_;
    }

    if (!LvxFileHandle::PrepareDataRead()) {
      state_ = kLvxFileDataInfoFault;
      return state_;
    }
  } else {
    lvx_file_.open(filename, mode | std::ios_base::binary);

    if (!lvx_file_.is_open()) {
      state_ = kLvxFileNotExist;
      return state_;
    }
  }

  return state_;
}

bool LvxFileHandle::Eof() {
  return lvx_file_.eof();
}

int LvxFileHandle::InitLvxFile() {
  time_t curtime = time(nullptr);
  char filename[30] = { 0 };

  tm* local_time = localtime(&curtime);
  sprintf(filename, "%d-%02d-%02d_%02d-%02d-%02d.lvx", local_time->tm_year + 1900,
                                                       local_time->tm_mon + 1,
                                                       local_time->tm_mday,
                                                       local_time->tm_hour,
                                                       local_time->tm_min,
                                                       local_time->tm_sec);

  return LvxFileHandle::Open(filename, std::ios::out | std::ios::binary);
}

void LvxFileHandle::InitLvxFileHeader() {
  LvxFileHeader lvx_file_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);
  std::string signature = LVX_HEADER_SIG_STR;
  memcpy(lvx_file_header.signature, signature.c_str(), signature.size());

  lvx_file_header.version[0] = 1;
  lvx_file_header.version[1] = 0;
  lvx_file_header.version[2] = 0;
  lvx_file_header.version[3] = 0;

  lvx_file_header.magic_code = MAGIC_CODE;

  memcpy(write_buffer.get() + cur_offset_, (void *)&lvx_file_header, sizeof(LvxFileHeader));
  cur_offset_ += sizeof(LvxFileHeader);

  uint8_t device_count = static_cast<uint8_t>(device_info_list_.size());
  memcpy(write_buffer.get() + cur_offset_, (void *)&device_count, sizeof(uint8_t));
  cur_offset_ += sizeof(uint8_t);

  for (int i = 0; i < device_count; i++) {
    memcpy(write_buffer.get() + cur_offset_, (void *)&device_info_list_[i], sizeof(LvxDeviceInfo));
    cur_offset_ += sizeof(LvxDeviceInfo);
  }

  lvx_file_.write((char *)write_buffer.get(), cur_offset_);
}

void LvxFileHandle::SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp) {
  uint64_t cur_pos = 0;
  FrameHeader frame_header = { 0 };
  std::unique_ptr<char[]> write_buffer(new char[WRITE_BUFFER_LEN]);

  int pack_num = point_packet_list_temp.size();
  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + (int64_t)pack_num * sizeof(LvxBasePackDetail) + sizeof(FrameHeader);
  frame_header.package_count = pack_num;
  frame_header.frame_index = cur_frame_index_;

  memcpy(write_buffer.get() + cur_pos, (void*)&frame_header, sizeof(FrameHeader));
  cur_pos += sizeof(FrameHeader);

  auto iter = point_packet_list_temp.begin();
  for (; iter != point_packet_list_temp.end(); iter++) {
    if (cur_pos + sizeof(LvxBasePackDetail) >= WRITE_BUFFER_LEN) {
      lvx_file_.write((char*)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), sizeof(LvxBasePackDetail));
      cur_pos += sizeof(LvxBasePackDetail);
    }
    else {
      memcpy(write_buffer.get() + cur_pos, (void*)&(*iter), sizeof(LvxBasePackDetail));
      cur_pos += sizeof(LvxBasePackDetail);
    }
  }
  lvx_file_.write((char*)write_buffer.get(), cur_pos);

  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileHandle::CloseLvxFile() {
  if (lvx_file_.is_open())
    lvx_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet) {
  packet.version = data->version;
  packet.port_id = data->slot;
  packet.lidar_index = data->id;
  packet.rsvd = data->rsvd;
  packet.error_code = data->err_code;
  packet.timestamp_type = data->timestamp_type;
  packet.data_type = data->data_type;
  memcpy(packet.timestamp, data->timestamp, 8 * sizeof(uint8_t));

  if (packet.data_type == 0) {
    LivoxRawPoint tmp[PACK_POINT_NUM];
    memcpy(tmp, (void *)data->data, PACK_POINT_NUM * sizeof(LivoxRawPoint));
    for (int i = 0; i < PACK_POINT_NUM; i++) {
      packet.point[i].x = static_cast<float>(tmp[i].x / 1000.0);
      packet.point[i].y = static_cast<float>(tmp[i].y / 1000.0);
      packet.point[i].z = static_cast<float>(tmp[i].z / 1000.0);
      packet.point[i].reflectivity = tmp[i].reflectivity;
    }
  } else if (packet.data_type == 1) {
    LivoxSpherPoint tmp[PACK_POINT_NUM];
    memcpy(tmp, (void *)data->data, PACK_POINT_NUM * sizeof(LivoxSpherPoint));
    for (int i = 0; i < PACK_POINT_NUM; i++) {
      packet.point[i].x = static_cast<float>(tmp[i].depth / 1000.0);
      packet.point[i].y = static_cast<float>(tmp[i].theta);
      packet.point[i].z = static_cast<float>(tmp[i].phi);
      packet.point[i].reflectivity = tmp[i].reflectivity;
      LivoxPoint temp = { packet.point[i].x * std::sin(packet.point[i].y) * std::cos(packet.point[i].z), packet.point[i].x * std::sin(packet.point[i].y) * std::sin(packet.point[i].z), packet.point[i].x * std::cos(packet.point[i].y) };
      packet.point[i] = temp;
    }
  }
}

int LvxFileHandle::GetFrame(LvxFrame *one_frame) {
  if (lvx_file_.eof()) {
    state_ = kLvxFileAtEnd;
    return kLvxFileAtEnd;
  }

  lvx_file_.read((char *)one_frame, sizeof(FrameHeader));

  if ((size_ < one_frame->header.current_offset) || (size_ < one_frame->header.next_offset)) {
    return kLvxFileUndefFault;
  }

  lvx_file_.read((char *)(one_frame->packet), one_frame->header.package_count * sizeof(LvxBasePackDetail));

  return 0;
}

int LvxFileHandle::GetLvxFileReadProgress() {
  if (!size_) {
    return 0;
  }

  if (!lvx_file_.eof()) {
    return (lvx_file_.tellg() * 100ULL) / size_;
  } else {
    return 100;
  }
}



