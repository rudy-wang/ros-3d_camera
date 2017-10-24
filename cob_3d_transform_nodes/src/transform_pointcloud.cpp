/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_driver
 * \note
 *  ROS package name: cob_env_model
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: September 2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// ROS core
#include <ros/ros.h>
#include <tf/transform_listener.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>


using namespace std;

class TransformPointCloud
{
  protected:
    ros::NodeHandle nh_;

  public:
    string cloud_topic_sub_;
    string cloud_topic_pub_;
    string target_frame_;

    tf::TransformListener tf_listener_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    TransformPointCloud() :
      nh_("~")
    {
      cloud_topic_sub_="input";
      cloud_topic_pub_="output";
      target_frame_ = "/map";

      pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_pub_,1);
      sub_ = nh_.subscribe (cloud_topic_sub_, 1,  &TransformPointCloud::cloud_cb, this);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
      sensor_msgs::PointCloud2 cloud_out;
      pcl_ros::transformPointCloud (target_frame_, *cloud, cloud_out, tf_listener_);
      cloud_out.header.stamp = cloud->header.stamp;
      pub_.publish(cloud_out);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "transform_pointcloud", ros::init_options::AnonymousName);

  TransformPointCloud b;
  ros::spin ();

  return (0);
}
/* ]--- */
