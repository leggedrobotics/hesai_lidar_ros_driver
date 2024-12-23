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
 * File: source_driver_ros1.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS1
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <ros/ros.h>
#include <csignal>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "std_msgs/UInt8MultiArray.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include "hesai_ros_driver/UdpFrame.h"
#include "hesai_ros_driver/UdpPacket.h"
#include "hesai_ros_driver/LossPacket.h"
#include "hesai_ros_driver/Ptp.h"
#include "hesai_ros_driver/Firetime.h"
#include <boost/thread.hpp>
#include "source_drive_common.hpp"

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init(const YAML::Node& config);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos1() {
    ros::MultiThreadedSpinner spinner(2); 
    spinner.spin();
  }
  rosbag::Bag outputBag;
  mutable std::mutex rosbagMutex_;
  ros::Time bagStartTime_;
  ros::Time bagEndTime_;
  ros::Time lastReceivedPacketRosTime_;
  // ros::Time lastlastReceivedPacketRosTime_;
  uint64_t duration_ = 0;
  std::string input_rosbag_path_ = std::string();
  std::mutex receive_packet_mutex_;
  uint64_t sequenceNumberPacket_ = 0;
  std::string outBagPath_ ="";
  uint64_t totalNumberOfPackets_ = 0;
  ros::Time lastPossibleMsgTime_;


  ros::Time latestCloudStamp_;
  bool save_replayed_topics_to_rosbag_ = false;
  bool save_additional_last_point_timestamp_cloud_ = false;

  std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;

protected:
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void RecieveCorrection(const std_msgs::UInt8MultiArray& msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void RecievePacket(const hesai_ros_driver::UdpFrame& msg);
  void RecievePTP(const hesai_ros_driver::Ptp& msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg, double);
  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the firetime correction 
  void SendFiretime(const double *firetime_correction_);
  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert point clouds into ROS messages
  sensor_msgs::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  // Convert double[512] to float64[512]
  hesai_ros_driver::Firetime ToRosMsg(const double *firetime_correction_);
  // publish point
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  // publish packet 
  ros::Publisher pkt_pub_;
  // packet sub
  ros::Subscriber pkt_sub_;
  ros::Subscriber ptp_sub_;
  //spin thread while recieve data from ROS topic
  boost::thread* subscription_spin_thread_;

  ros::Publisher crt_pub_;
  ros::Publisher firetime_pub_;
  ros::Publisher loss_pub_;
  ros::Publisher ptp_pub_;
  ros::Subscriber crt_sub_;
};


inline void SourceDriver::Init(const YAML::Node& config)
{
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  save_replayed_topics_to_rosbag_ = driver_param.input_param.save_replayed_topics_to_rosbag;

  save_additional_last_point_timestamp_cloud_ = driver_param.input_param.save_additional_last_point_timestamp_cloud;

  if (driver_param.input_param.send_point_cloud_ros) {
    pub_ = nh_->advertise<sensor_msgs::PointCloud2>(driver_param.input_param.ros_send_point_topic, 10);
  }
  
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = nh_->advertise<hesai_ros_driver::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  } 

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = nh_->advertise<hesai_ros_driver::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
    } 

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = nh_->advertise<std_msgs::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
    } 
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = nh_->advertise<hesai_ros_driver::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
    } 
  }

  if (driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET) {
    pkt_pub_ = nh_->advertise<hesai_ros_driver::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = nh_->subscribe(driver_param.input_param.ros_recv_packet_topic, 10000, &SourceDriver::RecievePacket, this);

    if (save_replayed_topics_to_rosbag_)
    {
      ptp_sub_ = nh_->subscribe(driver_param.input_param.ros_send_ptp_topic, 10000, &SourceDriver::RecievePTP, this);
    }
    

    if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
      crt_sub_ = nh_->subscribe(driver_param.input_param.ros_recv_correction_topic, 10, &SourceDriver::RecieveCorrection, this);
    }

    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos1,this));
  }

  driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
  driver_param.decoder_param.enable_parser_thread = true;

  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPointCloud, this, std::placeholders::_1));
  if(driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET){
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
  }
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
    }
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
    }
  } 
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
  // We use sim time true so should be okay.
  latestCloudStamp_ = ros::Time::now();
  lastPossibleMsgTime_ = ros::Time(0);
  // lastReceivedPacketRosTime_ = ros::Time(0);
  // lastlastReceivedPacketRosTime_ = ros::Time(0);
  // print ros time now
  std::cout << "ros time now:" << latestCloudStamp_ << std::endl;

  if (save_replayed_topics_to_rosbag_){

    ROS_INFO_STREAM("\033[92m"
                << " In the replay mode. Sleeping for a second to allow the rosbag to start playing."
                << "\033[0m");
    const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(0.5)};
    ros::WallTime::sleepUntil(first);
 
    // Get the directory of the ros package
    // std::string packagePath = ros::package::getPath("hesai_ros_driver");
      
    // Generate the log file name
    std::string outBagDirectory_ = ros::package::getPath("hesai_ros_driver") + "/data/";
    std::string outBagName_ = "hesaiXT32_" + std::to_string(ros::Time::now().toNSec()) + ".bag";

    if (driver_param.input_param.output_rosbag_directory != "") {
      outBagPath_ = driver_param.input_param.output_rosbag_directory + outBagName_;
    }else{
      outBagPath_ = outBagDirectory_ + outBagName_;
    }

    std::cout << "outBagPath_: " << outBagPath_ << std::endl;

    // Remove the old bag file
    if (std::filesystem::exists(outBagPath_.c_str()))
    {
      std::remove(outBagPath_.c_str());
    }

    // Open the new bag file
    outputBag.open(outBagPath_, rosbag::bagmode::Write); 
    outputBag.setCompression(rosbag::compression::LZ4);
    std::filesystem::permissions(
        outBagPath_,
        std::filesystem::perms::owner_all | std::filesystem::perms::group_all,
        std::filesystem::perm_options::add
    );

  }

}

inline void SourceDriver::Start()
{
  driver_ptr_->Start();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  input_rosbag_path_ = driver_ptr_->inputRosbagPath_;
  if (!(save_replayed_topics_to_rosbag_))
  {
    ROS_WARN("[HesaiXT32] Replay mode is off.");
   return;
  }

  if ( (input_rosbag_path_ != "") && (save_replayed_topics_to_rosbag_)){
    
    // ROS_WARN_STREAM("driver_ptr_->inputRosbagPath_: " << driver_ptr_->inputRosbagPath_);
    ROS_WARN_STREAM("Input rosbag path: " << input_rosbag_path_);

    // We need to do a dry-run on the rosbag to get the end time of the rosbag.
    rosbag::Bag inputBag;
    inputBag.open(input_rosbag_path_, rosbag::bagmode::Read);
    std::vector<std::string> viewtopics;
    viewtopics.push_back("/gt_box/hesai/packets");

    {
      rosbag::View view(inputBag, rosbag::TopicQuery(viewtopics));

      if (view.size() == 0) {
        ROS_WARN_STREAM("No messages found on " << viewtopics.front());
        return;
      }

      totalNumberOfPackets_ = view.size();

      auto first_it = view.begin();
      if (first_it != view.end())
      {
        // Instantiate as your actual message type
        hesai_ros_driver::UdpFrame::ConstPtr first_msg = first_it->instantiate<hesai_ros_driver::UdpFrame>();
        if (first_msg) {
          ROS_INFO_STREAM("First message header stamp = " << first_msg->header.stamp << " s");
          bagStartTime_ = first_msg->header.stamp;
        } else {
          ROS_WARN_STREAM("Could not instantiate the first message.");
        }
      }

      // -------------------------
      // 2. Check the last message
      // -------------------------
      auto it = view.begin();

      rosbag::View::iterator last_item;
      rosbag::View::iterator lastlast_item;
      while (it != view.end())
      {
        last_item = it++;

        if (it == view.end())
        {
          break;
        }
      }
      hesai_ros_driver::UdpFrame::ConstPtr last_msg = last_item->instantiate<hesai_ros_driver::UdpFrame>();
      ROS_INFO_STREAM("Last message header stamp  = " << last_msg->header.stamp << " s");
      lastPossibleMsgTime_ = last_msg->header.stamp;
      bagEndTime_ = last_msg->header.stamp;

      ros::Duration rosbag_duration = bagEndTime_ - bagStartTime_;
      duration_ = rosbag_duration.toNSec();
      ROS_WARN_STREAM("Duration: " << rosbag_duration.toSec()  << " s");

    }

  }else{
    ROS_ERROR_STREAM("Input rosbag path is empty. Exiting...");
    raise(SIGINT);
  }

}

inline SourceDriver::~SourceDriver()
{
  std::cout << "Stoping...." << std::endl;
  Stop();
}

inline void SourceDriver::Stop()
{
  std::string newName = input_rosbag_path_;
  {
    std::lock_guard<std::mutex> lock(rosbagMutex_);

    if (outputBag.isOpen())
    {
      outputBag.close();
      std::cout << "Rosbag Closed." << std::endl;
    }
  }

  {
    std::lock_guard<std::mutex> lock(rosbagMutex_);
    std::filesystem::path oldPath = std::filesystem::absolute(outBagPath_);

    if ( std::filesystem::exists( oldPath ) ){
      // Remove the `.bag` extension and add the new extension
      newName.erase(newName.end() - 4, newName.end());
      newName = newName + "_post_processed.bag";
      
      std::filesystem::path newPath = std::filesystem::absolute(newName);
      std::cout << "New Rosbag Path: " << newName << std::endl;
      std::cout << "Originally Recorded Path: " << outBagPath_ << std::endl;
      // std::filesystem::rename(oldPath, newPath);

      try {
        std::filesystem::rename(oldPath, newPath);
      } catch (std::filesystem::filesystem_error& e) {
        std::cout << e.what() << '\n';

        std::cout << "Can't move the file and rename. Instead will copy." << '\n';

        if (std::filesystem::exists( newPath ))
        {
          std::filesystem::remove(newPath);
        }

        std::filesystem::copy(oldPath,newPath);
        std::filesystem::remove(oldPath);
      }

    }
  }

  driver_ptr_->Stop();
}

inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_.publish(ToRosMsg(msg, timestamp));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  pub_.publish(ToRosMsg(msg, frame_id_));

  if (driver_ptr_->lidar_ptr_->rosbagEnded_ == true)
  {

    ROS_INFO_STREAM("\033[92m"
                  << "SUCCESSFULLY COMPLETED REPLAYING. TERMINATING MYSELF. "
                  << "\033[0m");

    raise(SIGINT);
  }
  
}

inline void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_.publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_.publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_.publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

inline void SourceDriver::SendFiretime(const double *firetime_correction_)
{
  firetime_pub_.publish(ToRosMsg(firetime_correction_));
}

inline sensor_msgs::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::PointCloud2 ros_msg;


  // Rarely, we have overflow issue due to reflected points or edge duplications. We truncate them.
  uint32_t frameSize = frame.points_num;
  if (save_replayed_topics_to_rosbag_)
  {
    if (frameSize > 64000)
    {
      frameSize = 64000;
    }
  }

  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.width = frameSize; 
  ros_msg.height = 1; 

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(frameSize * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
  for (size_t i = 0; i < frameSize; i++)
  {
    LidarPointXYZIRT point = frame.points[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;   
  }
    // ros_msg.header.seq = s;

  latestCloudStamp_ = ros::Time().fromSec(frame.points[0].timestamp);
  ros::Time lastPointStamp_ = ros::Time().fromSec(frame.points[frame.points_num - 1].timestamp);
    
  ros_msg.header.stamp = latestCloudStamp_;
  ros_msg.header.frame_id = frame_id_;

  sensor_msgs::PointCloud2 ros_msg_last = ros_msg;
  ros_msg_last.header.stamp = lastPointStamp_;

  {
    std::lock_guard<std::mutex> lock(receive_packet_mutex_);
    ros_msg.header.seq = sequenceNumberPacket_;
  }
  ros_msg_last.header.seq = ros_msg.header.seq;

  if (save_replayed_topics_to_rosbag_)
  {

    {
      std::lock_guard<std::mutex> lock(rosbagMutex_);
      outputBag.write("/gt_box/hesai/points", latestCloudStamp_, ros_msg);
    }

    if (save_additional_last_point_timestamp_cloud_)
    {
      {
        std::lock_guard<std::mutex> lock(rosbagMutex_);
        outputBag.write("/gt_box/hesai/points_last", lastPointStamp_, ros_msg_last);
      }
    }

    if ( ( (frame.frame_index % 50 == 0)  ||  (totalNumberOfPackets_ - frame.frame_index) < 11 ) && (frame.frame_index != 0))
    {
      printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;

      uint64_t m_time = latestCloudStamp_.toNSec();
      if ( (duration_ > 0) && (bagStartTime_.toNSec() > 0))
      {
        float progress = (float)(m_time - bagStartTime_.toNSec()) / (float)duration_ * 100;
        ROS_INFO("Processing PC message ( %.2f%% )", progress);
      }

      ROS_INFO_STREAM("Processed Message Ratio: " << frame.frame_index + 1 << "/" << totalNumberOfPackets_);
    }

    // ROS_INFO_STREAM("lastReceivedPacketRosTime_: " << lastReceivedPacketRosTime_);
    // ROS_INFO_STREAM("lastPossibleMsgTime_: " << lastPossibleMsgTime_);
    // ROS_INFO_STREAM("DIFF: " << (lastPossibleMsgTime_ - lastReceivedPacketRosTime_).toNSec());

    if ( (duration_ > 0) && (frame.frame_index > 1 ) )
    {
      if (lastReceivedPacketRosTime_.toNSec() > lastPossibleMsgTime_.toNSec())
      {
         ROS_ERROR("The Hesai Computer time rolled back in time while recording. i.e. the header time of the packet is not within the bag. Exitting.");
         ROS_ERROR_STREAM("Received Packet: " << lastReceivedPacketRosTime_ << " Last Possible Packet: " << lastPossibleMsgTime_);
         raise(SIGINT);
      }
      
      ros::Duration difff = lastPossibleMsgTime_ - lastReceivedPacketRosTime_;
      // ROS_INFO_STREAM("DIFF: " << difff.toNSec());
      if ((difff.toNSec()) < ( 10 * 1000 * 1000) && lastPossibleMsgTime_ != ros::Time(0))
      {
        ROS_INFO_STREAM("\033[92m" << "Rosbag replaying has been completed." << "\033[0m");
        ROS_INFO_STREAM("Last Received Packet Timestamp: " << lastReceivedPacketRosTime_);
        ROS_INFO_STREAM("Last Packet time from the bag : " << lastPossibleMsgTime_);
        ROS_INFO_STREAM("Time Difference (Expect ~ 0s) : " << difff);

        // +1 because the index is starting from 0. 
        ROS_INFO_STREAM("\033[92m" << "Final Processed Message Ratio: " << frame.frame_index + 1 << "/" << totalNumberOfPackets_ << "\033[0m");

        driver_ptr_->lidar_ptr_->rosbagEnded_ = true;
      }
    }
  }

  return ros_msg;
}

inline hesai_ros_driver::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  rs_msg.header.stamp = ros::Time().fromSec(timestamp);
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

inline std_msgs::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  std_msgs::UInt8MultiArray msg;
  msg.data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg.data.begin());
  return msg;
}

inline hesai_ros_driver::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;  
  if (save_replayed_topics_to_rosbag_)
  {
    {
      std::lock_guard<std::mutex> lock(rosbagMutex_);
      outputBag.write("/gt_box/hesai/packet_loss", latestCloudStamp_, msg);
    }
  }
  return msg;
}

inline hesai_ros_driver::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
  return msg;
}

inline hesai_ros_driver::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}

inline void SourceDriver::RecievePTP(const hesai_ros_driver::Ptp& msg){

  hesai_ros_driver::Ptp newMsg = msg;

  if (save_replayed_topics_to_rosbag_)
  {
    
    if (!outputBag.isOpen())
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(rosbagMutex_);
      outputBag.write("/gt_box/hesai/ptp", latestCloudStamp_, newMsg);
    }
  }
}

inline void SourceDriver::RecievePacket(const hesai_ros_driver::UdpFrame& msg)
{

  {
    std::lock_guard<std::mutex> lock(receive_packet_mutex_);
    sequenceNumberPacket_ = msg.header.seq;
    lastReceivedPacketRosTime_ = msg.header.stamp;

    for (size_t i = 0; i < msg.packets.size(); i++) {
      driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg.packets[i].data[0], msg.packets[i].size);
    }
  }

  // printf("Receiving Buffer Size:%lu\n",driver_ptr_->lidar_ptr_->origin_packets_buffer_.size());
}

inline void SourceDriver::RecieveCorrection(const std_msgs::UInt8MultiArray& msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg.data.size());
  std::copy(msg.data.begin(), msg.data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}
