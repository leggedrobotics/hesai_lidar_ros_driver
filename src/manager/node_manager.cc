/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of
   other contributors maybe used to endorse or promote products derived from this software without
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: node_manager.cc
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: ROS Node Manager
 * Created on June 12, 2023, 10:46 AM
 */

#include "manager/node_manager.h"
void NodeManager::Init(const YAML::Node& config)
{
  YAML::Node lidar_config = YamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    std::shared_ptr<SourceDriver> source;
    source = std::make_shared<SourceDriver>(SourceType::DATA_FROM_LIDAR);
    source->Init(lidar_config[i]);
    sources_driver_.emplace_back(source);
  }
}

void NodeManager::Start()
{
  for (auto& iter : sources_driver_)
  {
    if (iter != nullptr)
    {
      iter->Start();
    }
  }
}

void NodeManager::Stop()
{
  for (auto& iter : sources_driver_)
  {
    if (iter != nullptr)
    {
      iter->Stop();
    }
  }
}

NodeManager::~NodeManager()
{
  Stop();
}
std::vector<SourceDriver::Ptr> NodeManager::GetSourcesDriver()
{
  return sources_driver_;
}

bool NodeManager::IsPlayEnded()
{
  int num = GetSourcesDriver().size();
  bool all_pcap_end = true;

  // for (int i = 0; i < num; i++) {
  // all_pcap_end = GetSourcesDriver()[0]->driver_ptr_->lidar_ptr_->IsPlayEnded();
  all_pcap_end = GetSourcesDriver()[0]->driver_ptr_->lidar_ptr_->rosbagEnded_;

  // if (!all_pcap_end) {
  //   break;
  // }
  // }

  if (all_pcap_end)
  {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    printf("-----------------%d pcap(s) end, we will close the node(s)!!!-----------------\n", num);
    // if (system("rosnode kill rviz") == -1) {
    //   printf("Command Execution Error: rosnode kill rviz\n");
    //   all_pcap_end = false;
    // }
  }

  return all_pcap_end;
}

void NodeManager::SetRosbagReplayPath(const std::string& path)
{
  // if (GetSourcesDriver().size() == 1)
  // {
  printf("-----------------Setting the Input Rosbag Path!-----------------\n");
  GetSourcesDriver()[0]->driver_ptr_->inputRosbagPath_ = path;
  // }

  return;
}