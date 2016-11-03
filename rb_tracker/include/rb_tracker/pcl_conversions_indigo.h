/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Open Source Robotics Foundation, Inc. nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INDIGO_PCL_CONVERSIONS_H__
#define INDIGO_PCL_CONVERSIONS_H__

#include <vector>

#include <ros/ros.h>

#include <pcl/conversions.h>

#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>

#include <pcl/PCLImage.h>
#include <sensor_msgs/Image.h>

#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PointIndices.h>
#include <pcl_msgs/PointIndices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <pcl/Vertices.h>
#include <pcl_msgs/Vertices.h>

#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>

#include <pcl/io/pcd_io.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace pcl_conversions {

  /** PCLHeader <=> Header **/

  inline
  void fromPCL(const pcl::uint64_t &pcl_stamp, ros::Time &stamp)
  {
    stamp.fromNSec(pcl_stamp * 1e3);  // Convert from us to ns
  }

  inline
  void toPCL(const ros::Time &stamp, pcl::uint64_t &pcl_stamp)
  {
    pcl_stamp = stamp.toNSec() / 1e3;  // Convert from ns to us
  }

  inline
  ros::Time fromPCL(const pcl::uint64_t &pcl_stamp)
  {
    ros::Time stamp;
    fromPCL(pcl_stamp, stamp);
    return stamp;
  }

  inline
  pcl::uint64_t toPCL(const ros::Time &stamp)
  {
    pcl::uint64_t pcl_stamp;
    toPCL(stamp, pcl_stamp);
    return pcl_stamp;
  }

  /** PCLHeader <=> Header **/

  //inline
  //void fromPCL(const pcl::PCLHeader &pcl_header, std_msgs::Header &header)
  //{
    //fromPCL(pcl_header.stamp, header.stamp);
    //header.seq = pcl_header.seq;
    //header.frame_id = pcl_header.frame_id;
  //}

  //inline
  //void toPCL(const std_msgs::Header &header, pcl::PCLHeader &pcl_header)
  //{
    //toPCL(header.stamp, pcl_header.stamp);
    //pcl_header.seq = header.seq;
    //pcl_header.frame_id = header.frame_id;
  //}

  //inline
  //std_msgs::Header fromPCL(const pcl::PCLHeader &pcl_header)
  //{
    //std_msgs::Header header;
    //fromPCL(pcl_header, header);
    //return header;
  //}

  //inline
  //pcl::PCLHeader toPCL(const std_msgs::Header &header)
  //{
    //pcl::PCLHeader pcl_header;
    //toPCL(header, pcl_header);
    //return pcl_header;
  //}

}

#endif
