/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: dp_ptu47_trigger.cpp 32647 2010-03-03 18:48:59Z thibaux $
 *
 */

/**
@mainpage

\author Radu Bogdan Rusu

@b dp_ptu47_trigger triggers the DirectedPerception Pan-Tilt-Unit (PTU-47) service.
**/

#include <ros/ros.h>
#include "dp_ptu47_pan_tilt_stage/SendCommand.h"
#include "dp_ptu47_pan_tilt_stage/SendTrajectory.h"
#include "dp_ptu47_pan_tilt_stage/GetLimits.h"

using namespace dp_ptu47_pan_tilt_stage;

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR ("Syntax is: %s [pan_angle tilt_angle]... tolerance", argv[0]);
    return (-1);
  }

  ros::init (argc, argv, "dp_ptu47_trigger", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  ros::ServiceClient client_sc = n.serviceClient<SendCommand>("/dp_ptu47/control");
  ros::ServiceClient client_st = n.serviceClient<SendTrajectory>("/dp_ptu47/control_trajectory");
  ros::ServiceClient client_gl = n.serviceClient<GetLimits>("/dp_ptu47/get_limits");
  GetLimits srv_gl;
  client_gl.call (srv_gl);

  std::vector<float> pan_angles;
  std::vector<float> tilt_angles;
  for (int i = 0; 2*i + 2 < argc; i++) {
    pan_angles.push_back(atof(argv[2*i+1]));
    tilt_angles.push_back(atof(argv[2*i+2]));
    if (pan_angles[i] <= srv_gl.response.min_pan || pan_angles[i] >= srv_gl.response.max_pan)
    {
      ROS_ERROR ("Invalid pan angle specified. Limits are: %f -> %f.", srv_gl.response.min_pan, srv_gl.response.max_pan);
      return (-1);
    }
    if (tilt_angles[i] <= srv_gl.response.min_tilt || tilt_angles[i] >= srv_gl.response.max_tilt)
    {
      ROS_ERROR ("Invalid tilt angle specified. Limits are: %f -> %f.", srv_gl.response.min_tilt, srv_gl.response.max_tilt);
      return (-1);
    }
  }

  if (pan_angles.size() == 1)
  {
    SendCommand srv_sc;
    srv_sc.request.pan_angle = pan_angles[0];
    srv_sc.request.tilt_angle = tilt_angles[0];
    srv_sc.request.wait_finished = true;
    if (client_sc.call (srv_sc))
    {
      ROS_INFO ("Service call successful. New pan/tilt positions are: %f/%f.", srv_sc.response.pan_angle, srv_sc.response.tilt_angle);
    }
    else
    {
      ROS_ERROR ("Failed to call service!");
      return (-1);
    }
  }
  else
  {
    float tolerance = 0;
    if (argc % 2 == 0)
    {
      tolerance = atof(argv[argc-1]);
    }
    SendTrajectory srv_st;
    srv_st.request.pan_angles = pan_angles;
    srv_st.request.tilt_angles = tilt_angles;
    srv_st.request.tolerance = tolerance;
    if (client_st.call (srv_st))
    {
      ROS_INFO ("Trajectory service call successful. New pan/tilt positions are: %f/%f.", srv_st.response.pan_angle, srv_st.response.tilt_angle);
    }
    else
    {
      ROS_ERROR ("Failed to call service!");
      return (-1);
    }    
  }
  return (0);
}
