//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#include <Windows.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono> // For timestamps 
#include <iostream>
#include <fstream> // For file streams
#include <iomanip> // for std::put_time
#include <sstream> // for string streams
#include <vector>
#include <csignal> // Catch Cntrl + C. Safely close while loop. 

// constexpr double EPS = 0.15; 
// constexpr int MIN_PTS = 6; DBSCAN now runs offline. 

std::ofstream lidar_raw_binary; 
volatile sig_atomic_t keep_running = 1; 

#pragma pack(push, 1)
struct RawPoint {
    double timestamp; 
    float x, y, z; 
};
#pragma pack(pop)

// Signal Handler for Ctrl + C
void signal_int_handler(int signal_num) {
    printf("\n[Lidar Logger] Ctrl + C detected. Shutting down safely. \n"); 
    keep_running = 0; // Break the while() loop in main 
}

// Unix Timestamp Generator 
double now_unix() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    return duration_cast<microseconds>(duration).count() / 1e6; 
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr || !lidar_raw_binary.is_open()) {
        return;
    } 

    // 1. Grab the timestamp the EXACT moment the packet is processed 
    double packet_timestamp = now_unix(); 

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint* p_point_data = (LivoxLidarCartesianHighRawPoint*)data->data;

        // 2. Create a temporary buffer for just the packet's points 
        std::vector<RawPoint> packet_buffer; 
        packet_buffer.reserve(data->dot_num); // Allocate exactly the required memory using .reserve()

        // 3. Go through every point in the packet. Convert mm to m.
        for (uint32_t i = 0; i < data->dot_num; ++i) {
            // for every point, create a RawPoint struct. 
            // all points in this packet have the same timestamp 
            RawPoint raw_pt;
            raw_pt.timestamp = packet_timestamp; 
            raw_pt.x = p_point_data[i].x / 1000.0f;
            raw_pt.y = p_point_data[i].y / 1000.0f;
            raw_pt.z = p_point_data[i].z / 1000.0f;
            packet_buffer.push_back(raw_pt); 
        }

        // 4. Write the entire packet to the binary file 
        // write() wants a string of characters. reinterpret_cast, pretends the floats are a string of bytes. 
        lidar_raw_binary.write(reinterpret_cast<const char*>(packet_buffer.data()), 
            packet_buffer.size() * sizeof(RawPoint));

    } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint* p_point_data = (LivoxLidarCartesianLowRawPoint*)data->data;
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint*)data->data;
    }
    printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

  // LivoxLidarIpInfo lidar_ip_info;
  // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
  // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}

int main(int argc, const char *argv[]) {
    if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
    }

    // Register the Ctrl + C signal handler 
    signal(SIGINT, signal_int_handler);

    const std::string path = argv[1];

    // REQUIRED, to init Livox SDK2
    if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
    }

    // 1. Get the current time
    auto now = std::chrono::system_clock::now(); // Grab motherboard clock
    auto in_time_t = std::chrono::system_clock::to_time_t(now); // Translate into time_t to create filename

    // 2. Format the time into a string (e.g., "2025_10_27-17_06_15")
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d--%H_%M_%S");
    std::string startup_time_string = ss.str();

    // 3. Create the full filename
    std::string filename = "lidar_raw_" + startup_time_string + ".bin";

    // 4. Open the uniquely named file
    // Ensure 20 bytes RawPt struct is 20 bytes long by telling windows it is binary
    lidar_raw_binary.open(filename, std::ios::binary);
    if (lidar_raw_binary.is_open()) {

        printf("Logging raw LiDAR data to %s\n", filename.c_str());
    }
    else {
        printf("ERROR: Could not open binary file for writing.\n");
    }
  
    // Tells Livox SDK (running in the background): 
    // Save address of func. When data comes in, YOU run this func.
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
    // OPTIONAL, to get imu data via 'ImuDataCallback'
    // some lidar types DO NOT contain an imu component
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  
    SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
    // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    // If and else are the same because: Windows uses 'Sleep'. Linux/Mac uses 'sleep'. 
    // Main ONLY runs once. However, this loop forces MAIN to sleep forever, as the SDK 
    // gathers the data from the lidar. Once Cntrl+C is pressed, we exit this loop and close the file. 
    while (keep_running) {
        #ifdef WIN32
            Sleep(100);
        #else
            usleep(100000); 
        #endif
    }

    if (lidar_raw_binary.is_open()) {
        lidar_raw_binary.close();
        printf("Raw Lidar binary file is closed.\n");
    }

    LivoxLidarSdkUninit();
    printf("Livox Quick Start Demo End!\n");
    return 0;
}
