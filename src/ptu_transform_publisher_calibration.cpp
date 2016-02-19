/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: ptu_transform_publisher_calibration.cpp 43793 2010-08-25 19:23:26Z rusu $
 *
 */

#include <dp_ptu47_pan_tilt_stage/PanTiltStamped.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

using namespace dp_ptu47_pan_tilt_stage;

/**
  * \author Romain Thibaux, Radu Bogdan Rusu
  *
  * @b listens for PanTilt messages and outputs corresponding transforms
  * from:
  * - /base_link to /ptu/base_link
  * - /ptu/base_link to /ptu/turret 
  * - /ptu/turret to /ptu/table
  */
class PanTiltTransformPublisherCalibration
{
  public:
    PanTiltTransformPublisherCalibration (const ros::NodeHandle &nh) : 
      nh_ (nh), stem_height_ (-1.0), table_size_ (-1.0)  
    {
      // Get the parameters from the server
      nh_.getParam ("/ptu/stem_height", stem_height_);
      nh_.getParam ("/ptu/table_size", table_size_);

      tf::Point rotation_center (0,0,0);
      tf::Quaternion rotation (0,0,0,0);
      nh_.getParam ("/ptu/center/x", rotation_center[0]);
      nh_.getParam ("/ptu/center/y", rotation_center[1]);
      nh_.getParam ("/ptu/center/z", rotation_center[2]);
      nh_.getParam ("/ptu/rotation/x", rotation[0]);
      nh_.getParam ("/ptu/rotation/y", rotation[1]);
      nh_.getParam ("/ptu/rotation/z", rotation[2]);
      nh_.getParam ("/ptu/rotation/w", rotation[3]);

      if (stem_height_ == -1.0 || table_size_ == -1.0)
      {
        ROS_ERROR ("Calibration values not present on the parameter server!");
        ros::shutdown ();
        return;
      }

      // Set the tf::Transform values
      head_to_ptu_.setOrigin (rotation_center);
      head_to_ptu_.setRotation (rotation);
      
      ptu_base_to_turret_.setOrigin (tf::Point (0, 0, 0));

      ROS_INFO ("Calibration parameters are:\n"
                 "- stem height: %f\n"
                 "- table size : %f\n"
                 "- center     : %f, %f, %f\n"
                 "- rotation   : %f, %f, %f, %f", stem_height_, table_size_, 
                 rotation_center[0], rotation_center[1], rotation_center[2],
                 rotation[0], rotation[1], rotation[2], rotation[3]);

      // Subscribe to the pan tilt status topic
      std::string pan_tilt_topic = nh_.resolveName ("pan_tilt_stamped");
      pan_tilt_sub_.subscribe (nh_, pan_tilt_topic, 100);
      pan_tilt_sub_.registerCallback (boost::bind (&PanTiltTransformPublisherCalibration::callback, this, _1));

      // Send the /base_link to /ptu/base_link transform every 0.01 seconds
      timer_  = nh_.createTimer (ros::Duration (0.01), boost::bind (&PanTiltTransformPublisherCalibration::timerCallback, this, _1));  // Call it every 0.01s after that

      // Prepare the marker
      table_marker_.header.frame_id = "/ptu/table";
      table_marker_.ns = "table_location";
      table_marker_.id = 0;
      table_marker_.type = visualization_msgs::Marker::LINE_STRIP;
      table_marker_.action = visualization_msgs::Marker::ADD;
      table_marker_.pose.position.x = 0; table_marker_.pose.position.y = 0; table_marker_.pose.position.z = 0;
      table_marker_.pose.orientation.x = 0.0; table_marker_.pose.orientation.y = 0.0; table_marker_.pose.orientation.z = 0.0; table_marker_.pose.orientation.w = 1.0;
      table_marker_.scale.x = 0.005;
      table_marker_.color.r = 0.0f; table_marker_.color.g = 1.0f; table_marker_.color.b = 0.0f; table_marker_.color.a = 1.0;
      double d = table_size_ / 2.0;
      
      geometry_msgs::Point p;
      p.z = 0;
      p.x = d;  p.y = d;  table_marker_.points.push_back (p);
                p.y = -d; table_marker_.points.push_back (p);
      p.x = -d;           table_marker_.points.push_back (p);
                p.y = d;  table_marker_.points.push_back (p);
      p.x = d;            table_marker_.points.push_back (p);

      // Prepare the PointCloud
      table_cloud_.header.frame_id = table_marker_.header.frame_id;
      table_cloud_.height = 1; table_cloud_.width = 4;
      table_cloud_.fields.resize (3); 
      table_cloud_.fields[0].name = "x"; table_cloud_.fields[0].offset = 0; table_cloud_.fields[0].datatype = 7; table_cloud_.fields[0].count = 1;
      table_cloud_.fields[1].name = "y"; table_cloud_.fields[1].offset = 4; table_cloud_.fields[1].datatype = 7; table_cloud_.fields[1].count = 1;
      table_cloud_.fields[2].name = "z"; table_cloud_.fields[2].offset = 8; table_cloud_.fields[2].datatype = 7; table_cloud_.fields[2].count = 1;
      table_cloud_.point_step = 12; table_cloud_.row_step = table_cloud_.point_step * table_cloud_.width;
      table_cloud_.is_dense = false;
      table_cloud_.data.resize (3 * table_cloud_.row_step);
      float val = 0;
      for (size_t i = 0; i < table_marker_.points.size (); ++i)
      {
        // We cannot assume that xyz are in order, so we'll memcpy
        val = table_marker_.points[i].x;
        memcpy (&table_cloud_.data[i * table_cloud_.point_step + table_cloud_.fields[0].offset], &val, sizeof (float));
        val = table_marker_.points[i].y;
        memcpy (&table_cloud_.data[i * table_cloud_.point_step + table_cloud_.fields[1].offset], &val, sizeof (float));
        val = table_marker_.points[i].z;
        memcpy (&table_cloud_.data[i * table_cloud_.point_step + table_cloud_.fields[2].offset], &val, sizeof (float));
      }

      marker_publisher_ = nh_.advertise<visualization_msgs::Marker> ("/ptu/table_marker", 0);
      cloud_publisher_  = nh_.advertise<sensor_msgs::PointCloud2> ("/ptu/table_hull", 0);
    };
 
  private:
    /** \brief Timer callback. */
    void 
      timerCallback (const ros::TimerEvent &timer_event)
    {
      // Send the transform
      transform_broadcaster_.sendTransform (tf::StampedTransform (head_to_ptu_, timer_event.current_expected, "/base_link", "/ptu/base_link"));
    }

    /** \brief Pan tilt status callback. */
    void 
      callback (const PanTiltStampedConstPtr &pan_tilt)
    {
      // Pan is counter_clockwise, the z axis is the vertical
      // We define the x axis as the tilt axis, with a 90 degree tilt bringing y to z
      double pan_radians = (pan_tilt->pan_angle / 180) * M_PI;
      double tilt_radians = (pan_tilt->tilt_angle / 180) * M_PI;

      // Set the transforms
      ptu_base_to_turret_.setRotation (tf::Quaternion (0, 0, sin (pan_radians / 2.0), cos (pan_radians / 2.0)));
      turret_to_table_.setOrigin (tf::Point (0, - stem_height_ * sin (tilt_radians), stem_height_ * cos (tilt_radians)));
      turret_to_table_.setRotation (tf::Quaternion (sin (tilt_radians / 2.0), 0, 0, cos (tilt_radians / 2.0)));

      // Send the transform
      transform_broadcaster_.sendTransform (tf::StampedTransform (ptu_base_to_turret_, pan_tilt->header.stamp, "/ptu/base_link", "/ptu/turret"));
      transform_broadcaster_.sendTransform (tf::StampedTransform (turret_to_table_, pan_tilt->header.stamp, "/ptu/turret", "/ptu/table"));

      // Show where we think the table is, useful for debugging
      table_cloud_.header.stamp  = pan_tilt->header.stamp;
      table_marker_.header.stamp = pan_tilt->header.stamp;
      marker_publisher_.publish (table_marker_);
      cloud_publisher_.publish (table_cloud_);

      // Update the parameter, in case it was changed by calibration
      nh_.getParam ("/ptu/stem_height", stem_height_);
    }

    ros::NodeHandle nh_;
    ros::Timer timer_;
    double stem_height_;  // Distance from the center of rotation to the center of the table
    double table_size_;   // Size of one side of the table (the table is square)

    message_filters::Subscriber<PanTiltStamped> pan_tilt_sub_;
    tf::TransformBroadcaster transform_broadcaster_;

    /** \brief The TF transform from /base_link to /ptu/base_link. */
    tf::Transform head_to_ptu_;
    
    /** \brief The TF transform from /ptu/base_link to /ptu/turret. */
    tf::Transform ptu_base_to_turret_;

    /** \brief The TF transform from /ptu/turret to /ptu/table. */
    tf::Transform turret_to_table_;

    sensor_msgs::PointCloud2 table_cloud_;
    // Debugging visualization
    visualization_msgs::Marker table_marker_;
    ros::Publisher marker_publisher_, cloud_publisher_;
};

int 
  main(int argc, char** argv)
{
  ros::init (argc, argv, "ptu_transform_publisher_calibration");
  ros::NodeHandle nh;

  PanTiltTransformPublisherCalibration publisher (nh);
  ros::spin ();
}
