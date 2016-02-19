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
 * $Id: dp_ptu47.cpp 43793 2010-08-25 19:23:26Z rusu $
 *
 */

/**
  * \author Radu Bogdan Rusu, Romain Thibaux
  * @b dp_ptu47 controls a DirectedPerception Pan-Tilt-Unit (PTU-47) unit using a ROS service.
  */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "dp_ptu47_pan_tilt_stage/SendCommand.h"
#include "dp_ptu47_pan_tilt_stage/SendTrajectory.h"
#include "dp_ptu47_pan_tilt_stage/DpPtu47Config.h"
#include "dp_ptu47_pan_tilt_stage/PanTilt.h"
#include "dp_ptu47_pan_tilt_stage/PanTiltStamped.h"
#include "dp_ptu47_pan_tilt_stage/GetLimits.h"
#include "dp_ptu47_pan_tilt_stage/Reset.h"

using namespace std;

class DpPtu47
{
  protected:
    ros::NodeHandle nh_;

  public:

    // ROS members
    ros::ServiceServer control_serv_, control_trajectory_serv_, get_limits_serv_, reset_serv_;
    ros::Publisher pub_;
    ros::Publisher stamped_pub_;

    int port_handle_;
    string port_name_;
    struct termios oldtio_;
    char buffer_[256];

    // The resolution of the unit (pan/tilt)
    float pan_resolution_, tilt_resolution_;
    // The actual stored limit values (pan/tilt)
    float unit_min_pan_, unit_max_pan_, unit_min_tilt_, unit_max_tilt_;
    // The user defined limits
    float user_min_pan_, user_max_pan_, user_min_tilt_, user_max_tilt_;
    bool limits_disabled_, init_;
    int reset_;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DpPtu47 () : nh_ ("~"), limits_disabled_ (false), init_ (false)
    {
      // Basic default parameters
      nh_.param ("port", port_name_, string ("/dev/ttyUSB0"));
      nh_.param ("reset_on_start", reset_, 0);
      if (reset_ != 0)
        ROS_INFO ("Reset on start enabled.");

      // Advertise the service that we want to use
      control_serv_ = nh_.advertiseService ("control", &DpPtu47::control2, this);
      control_trajectory_serv_ = nh_.advertiseService ("control_trajectory", &DpPtu47::controlTrajectory, this);
      get_limits_serv_ = nh_.advertiseService ("get_limits", &DpPtu47::get_limits, this);
      reset_serv_ = nh_.advertiseService ("reset", &DpPtu47::reset, this);

      pub_ = nh_.advertise<dp_ptu47_pan_tilt_stage::PanTilt> ("pan_tilt_status", 1);
      stamped_pub_ = nh_.advertise<dp_ptu47_pan_tilt_stage::PanTiltStamped> ("pan_tilt_status_stamped", 1);

      port_handle_ = -1;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~DpPtu47 ()
    {
      if (port_handle_ > 0)
      {
        ROS_INFO ("Closing the device.");
        tcsetattr (port_handle_, TCSANOW, &oldtio_);
        close (port_handle_);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Sends a command to the serial port and returns immediately
      * \param data the actual data buffer to write
      */
    bool
      send (const char *data)
    {
      if (port_handle_ < 0)
      {
        ROS_ERROR ("Invalid port handle!");
        return (false);
      }

      int length = strlen (data);

      // Write to port
      if (write (port_handle_, data, length) < length)
      {
        ROS_WARN ("Error writing to port!");
        return (false);
      }

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Sends a command to the serial port, reads the reply, and returns 
      * \param cmd the actual command to write
      * \param reply the buffer holding the reply
      */
    bool
      writeCmd (const char *cmd, char *reply)
    {
      // Send the command
      send (cmd);
      // Read the reply
      int len = read (port_handle_, reply, 255);
      // Check for errors
      if (len <= 0)               // no data read?
        return (false);
      if (reply[0] == '!')        // did we hit one of the limits?
      {
        if (reply[2] != '*')      // was the command executed successfully?
          return (false);
        else
          return (true);
      }
      if (reply[0] != '*')
        return (false);
      return (true);
     }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Sends a command to the serial port, reads the reply, and returns 
      * \param cmd the actual command to write
      * \param reply the buffer holding the reply
      */
    bool
    setAcceleration (int acceleration, char type)
    {
      char cmd[16];
      snprintf (cmd, 16, "%ca%d ", type, acceleration);
      
      // Send the command
      send (cmd);
      // Read the reply
      int count = 0;
      while (read (port_handle_, buffer_, 255) <= 0) {
	ros::Duration(0.1).sleep();
	if (count++ > 100) {
	  return false;
	}
      }
      ROS_WARN("Received:");
      for (int i = 0; buffer_[i] != '\r'; i++)
      {
	ROS_WARN("%c", buffer_[i]);
      }
      if (buffer_[0] == '!')        // did we hit one of the limits?
      {
	ROS_WARN("Limit hit while setting acceleration");
        if (buffer_[2] != '*')      // was the command executed successfully?
          return (false);
        else
          return (true);
      }
      if (buffer_[0] != '*')
      {
        return (false);
      }
      return (true);
     }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Initialize the PTU
      * @note: code based on the LGPL ptu46 Player driver (http://playerstage.sf.net)
      */
    bool
      init ()
    {
      ROS_INFO ("Opening unit on %s.", port_name_.c_str ());
      // Open the serial port
      port_handle_ = open (port_name_.c_str (), O_RDWR | O_NOCTTY | O_NDELAY);
      if (port_handle_ < 0)
      {
        ROS_ERROR ("Could not open serial port on %s.", port_name_.c_str ());
        return (false);
      }

      // File control - clear flags
      fcntl (port_handle_, F_SETFL, 0);

      // Save the current port I/O settings
      tcgetattr (port_handle_, &oldtio_);

      // Set up the new I/O settings
      struct termios newtio;
      memset (&newtio, 0, sizeof (newtio));
      newtio.c_cflag = CS8 | CLOCAL | CREAD;
      newtio.c_iflag = IGNPAR;
      newtio.c_oflag = 0;
      newtio.c_lflag = ICANON;

      speed_t sp = B9600;
      // Set the new baud rate and flags
      if (cfsetispeed (&newtio, sp) < 0 || cfsetospeed (&newtio, sp) < 0)
      {
        ROS_ERROR ("Failed to set the baud rate to 9600bps!");
        // Restore original I/O settings
        tcsetattr (port_handle_, TCSANOW, &oldtio_);
        close (port_handle_);
        port_handle_ = -1;
        return (false);
      }

      // Activate the new settings
      tcflush (port_handle_, TCIFLUSH);
      tcsetattr (port_handle_, TCSANOW, &newtio);

      // Set up the PTU
      send (" ");
      usleep (100000);
      tcflush (port_handle_, TCIFLUSH);

      // Disable echo and set ASCII terse feedback
      send ("ft ");
      send ("ed ");
      usleep (200000);
      tcflush (port_handle_, TCIFLUSH);

      // Get configuration parameters and update them via dynamic reconfigure
      tilt_resolution_ = getResolution ('t');
      pan_resolution_  = getResolution ('p');

      unit_min_pan_  = getLimit ('p', 'n');
      unit_max_pan_  = getLimit ('p', 'x');
      unit_min_tilt_ = getLimit ('t', 'n');
      unit_max_tilt_ = getLimit ('t', 'x');

      ROS_INFO ("Pan/tilt resolution: %f/%f, Pan limits: %d/%d (%f/%f), Tilt limits: %d/%d (%f/%f).", pan_resolution_, tilt_resolution_,
                int (unit_min_pan_ / pan_resolution_), int (unit_max_pan_ / pan_resolution_), unit_min_pan_, unit_max_pan_,
                int (unit_min_tilt_ / tilt_resolution_), int (unit_max_tilt_ / tilt_resolution_), unit_min_tilt_, unit_max_tilt_);
      if (tilt_resolution_ <= 0 || pan_resolution_ <= 0 || unit_min_pan_ == 0 || unit_max_pan_ == 0 || unit_min_tilt_ == 0 || unit_max_tilt_ == 0 || reset_ != 0)
      {
        send ("r ");

        // Wait for reset to complete
        int len = 0;
        char temp;
        char response[10] = "!T!T!P!P*";

        for (int i = 0; i < 9; ++i)
        {
          while ((len = read (port_handle_, &temp, 1)) == 0) ;
          if ((len != 1) || (temp != response[i]))
          {
            ROS_ERROR ("Error resetting the unit.");
            tcsetattr (port_handle_, TCSANOW, &oldtio_);
            close (port_handle_);
            port_handle_ = -1;
            return (false);
          }
        }

        // Delay here so data has arrived at serial port so we can flush it
        ros::Duration (1.0, 0).sleep ();
        tcflush (port_handle_, TCIFLUSH);

        // Get the limits again
        unit_min_pan_  = getLimit ('p', 'n');
        unit_max_pan_  = getLimit ('p', 'x');
        unit_min_tilt_ = getLimit ('t', 'n');
        unit_max_tilt_ = getLimit ('t', 'x');

        if (tilt_resolution_ <= 0 || pan_resolution_ <= 0 || unit_min_pan_ == 0 || unit_max_pan_ == 0 || unit_min_tilt_ == 0 || unit_max_tilt_ == 0)
        {
          ROS_ERROR ("Error getting pan/tilt resolution.");
          tcsetattr (port_handle_, TCSANOW, &oldtio_);
          close (port_handle_);
          port_handle_ = -1;
          return (false);
        }
      }

      tcflush (port_handle_, TCIFLUSH);
      init_ = true;
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Read the resolution of the pan/tilt in degrees
      * \param type the type of rotation/movement (pan = 'p', tilt = 't')
      */
    float
      getResolution (char type)
    {
      if (port_handle_ < 0)
        return (-1);

      char cmd[4] = " r ";
      cmd[0] = type;
      
      // Send the command
      send (cmd);
      // Read the reply
      int len = read (port_handle_, buffer_, 255);
      // Check for errors
      if (len < 3 || buffer_[0] != '*')
      {
        ROS_ERROR ("Error getting the pan/tilt resolution! (len = %d, buffer = %s)", len, buffer_);
        return (-1);
      }
      buffer_[len] = '\0';
      return (strtod (&buffer_[2], NULL) / 3600.0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Read the limits of the pan/tilt
      * \param type the type of rotation/movement (pan = 'p', tilt = 't')
      * \param type of limit (min = 'n', max = 'x')
      */
    float
      getLimit (char type, char limit)
    {
      if (port_handle_ < 0)
        return (-1);

      char cmd[4] = "   ";
      cmd[0] = type;
      cmd[1] = limit;

      // Send the command
      send (cmd);
      // Read the reply
      int len = read (port_handle_, buffer_, 255);
      // Check for errors
      if (len < 3 || buffer_[0] != '*')
      {
        ROS_ERROR ("Error getting the pan/tilt limits! (len = %d, buffer = %s)", len, buffer_);
        return (-1);
      }
      buffer_[len] = '\0';
      return (strtol (&buffer_[2], NULL, 0) * (type == 't' ? tilt_resolution_ : pan_resolution_));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Read the current pan/tilt position in degrees
      * \param type the type of rotation/movement (pan = 'p', tilt = 't')
      */
    float
      getPosition (char type)
    {
      return getIntPosition(type) * (type == 't' ? tilt_resolution_ : pan_resolution_);
    }

    int
      getIntPosition (char type)
    {
      if (port_handle_ < 0)
        return (-1);

      char cmd[4] = " p ";
      cmd[0] = type;

      // Send the command
      send (cmd);
      // Read the reply
      int len = read (port_handle_, buffer_, 255);
      // Check for errors
      if (len < 3 || buffer_[0] != '*')
      {
        ROS_ERROR ("Error getting the pan/tilt position! (len = %d, cmd = %s, buffer = %s)", len, cmd, buffer_);
        return (-1);
      }
      buffer_[len] = '\0';
      return strtod(&buffer_[2], NULL);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Set the pan/tilt position
      * \param type the type of rotation/movement (pan = 'p', tilt = 't')
      * \param position the desired angular position
      * \param wait_finished sets whether the command should block until the unit reached the desired position
      */
    bool
      setPosition (char type, float position, bool wait_finished = true)
    {
      if (port_handle_ < 0)
        return (false);

      // Get the RAW encoder count to move
      int count = static_cast<int> (position / (type == 't' ? tilt_resolution_ : pan_resolution_));

      // Check limits
      if (position < (type == 't' ? unit_min_tilt_ : unit_min_pan_) ||
          position > (type == 't' ? unit_max_tilt_ : unit_max_pan_))
      {
        ROS_ERROR ("Pan/Tilt value out of range: %c %f(%d) (%f/%f)", type, position, count,
                   (type == 't' ? unit_min_tilt_ : unit_min_pan_), (type == 't' ? unit_max_tilt_ : unit_max_pan_));
        return (false);
      }

      char cmd[16];
      snprintf (cmd, 16, "%cp%d ", type, count);
      // Send the actual position
      if (!writeCmd (cmd, buffer_))
      {
        ROS_ERROR ("Error setting pan/tilt position to: %f!", position);
        return (false);
      }

      // Should we wait until the desired position has been reached?
      if (wait_finished)
      {
        memset (buffer_, 0, 255);
        // Await position command completition
        if (!writeCmd ("a ", buffer_))
        {
          ROS_ERROR ("Error setting the unit into await position mode! Buffer: %s", buffer_);
          //return (false);
        }
        // Return to immediate mode
        if (!writeCmd ("i ", buffer_))
        {
          ROS_ERROR ("Error setting the unit back into immediate position mode! Buffer: %s", buffer_);
          //return (false);
        }
       }
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Set the new pan/tilt limits
      * \param type the type of rotation/movement (pan = 'p', tilt = 't')
      * \param type of limit (min = 'n', max = 'x')
      * \param new_limit the new limit in encoder counts
      */
    bool
      setLimit (char type, char limit, float new_limit)
    {
      if (port_handle_ < 0)
        return (false);

      // Get the RAW encoder count to move
      int count = static_cast<int> (new_limit / (type == 't' ? tilt_resolution_ : pan_resolution_));

      char cmd[16];
      snprintf (cmd, 16, "%cp%cp%d ", type, limit, count);
      printf ("%cp%cp%d \n", type, limit, count);
      // Send the actual value
      if (!writeCmd (cmd, buffer_))
      {
        ROS_ERROR ("Error setting the new pan/tilt limits (%d/%f)! %s", count, new_limit, buffer_);
        return (false);
      }

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Enable/disable limits
      * \param enable set to 'e' if unit limits should be enabled, 'd' otherwise
      */
    bool
      enableLimits (char enable)
    {
      if (port_handle_ < 0)
        return (-1);

      char cmd[4] = "l  ";
      cmd[1] = enable;

      if (!writeCmd (cmd, buffer_))
      {
        ROS_ERROR ("Error changing the limits status!");
        return (false);
      }
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Triggers a request via a service call */
    bool
      reset (dp_ptu47_pan_tilt_stage::Reset::Request &req, dp_ptu47_pan_tilt_stage::Reset::Response &resp)
    {
      send ("r ");

      // Wait for reset to complete
      int len = 0;
      char temp;
      char response[10] = "!T!T!P!P*";

      for (int i = 0; i < 9; ++i)
      {
        while ((len = read (port_handle_, &temp, 1)) == 0) ;
        if ((len != 1) || (temp != response[i]))
        {
          ROS_ERROR ("Error resetting the unit.");
          tcsetattr (port_handle_, TCSANOW, &oldtio_);
          close (port_handle_);
          port_handle_ = -1;
          return (false);
        }
      }

      // Delay here so data has arrived at serial port so we can flush it
      ros::Duration (1.0, 0).sleep ();
      tcflush (port_handle_, TCIFLUSH);
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Triggers a request via a service call */
    bool
      get_limits (dp_ptu47_pan_tilt_stage::GetLimits::Request &req, dp_ptu47_pan_tilt_stage::GetLimits::Response &resp)
    {
      // Get the current limits
      resp.min_pan = getLimit ('p', 'n');
      resp.max_pan = getLimit ('p', 'x');
      resp.min_tilt = getLimit ('t', 'n');
      resp.max_tilt = getLimit ('t', 'x');
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Triggers a request via a service call */
    bool
      control (dp_ptu47_pan_tilt_stage::SendCommand::Request &req, dp_ptu47_pan_tilt_stage::SendCommand::Response &resp)
    {
      // Set slaved position execution mode
      if (!writeCmd ("s ", buffer_))
        ROS_ERROR ("Error setting unit into slave position execution mode!");

      // Set the new values
      if (!setPosition ('p', req.pan_angle, false))
      {
        ROS_ERROR ("Error setting pan angle to %f.", req.pan_angle);
        return (false);
      }
      if (!setPosition ('t', req.tilt_angle, req.wait_finished))
      {
        ROS_ERROR ("Error setting tilt angle to %f.", req.tilt_angle);
        return (false);
      }

      resp.pan_angle  = getPosition ('p');
      resp.tilt_angle = getPosition ('t');

      ROS_INFO ("Setting a new pan/tilt position to: %f/%f", resp.pan_angle, resp.tilt_angle);

      // Send the data out as a message as well
      dp_ptu47_pan_tilt_stage::PanTilt pts;
      pts.pan_angle  = resp.pan_angle;
      pts.tilt_angle = resp.tilt_angle;
      pub_.publish (pts);
      return (true);
    }

    void publishPosition(int pan, int tilt)
    {
      // Publish a state stamped message with the current state (angle)
      dp_ptu47_pan_tilt_stage::PanTiltStamped pts;
      pts.pan_angle  = pan * pan_resolution_;
      pts.tilt_angle = tilt * tilt_resolution_;
      pts.header.stamp = ros::Time::now ();
      pts.header.frame_id = "/ptu/base_link";
      stamped_pub_.publish (pts);
    }

    bool setupSmoothMotion()
    {
      // Prepare motion parameters
      if (!writeCmd ("pb100 ", buffer_))
      {
	ROS_ERROR ("Error setting slow base speed!");
	return (false);
      }
      if (!setAcceleration(8000, 'p'))
      {
	ROS_ERROR ("Error setting slow acceleration!");
	return (false);
      }
      if (!writeCmd ("ps2000 ", buffer_))
      {
	ROS_ERROR ("Error setting speed!");
	return (false);
      }
      if (!writeCmd ("tb100 ", buffer_))
      {
	ROS_ERROR ("Error setting slow base speed!");
	return (false);
      }
      if (!setAcceleration(32000, 't'))
      {
	ROS_ERROR ("Error setting slow acceleration!");
	return (false);
      }
      if (!writeCmd ("ts4000 ", buffer_))
      {
	ROS_ERROR ("Error setting speed!");
	return (false);
      }
      return true;
    }

    bool checkPositionRequest(float pan, float tilt)
    {
      if ((pan < unit_min_pan_) || (pan > unit_max_pan_))
	{
	  ROS_ERROR ("Pan value out of range: %f (%f/%f)", pan, unit_min_pan_, unit_max_pan_);
	  return (false);
	}

      if ((tilt < unit_min_tilt_) || (tilt > unit_max_tilt_))
	{
	  ROS_ERROR ("Tilt value out of range: %f (%f/%f)", tilt, unit_min_tilt_, unit_max_tilt_);
	  return (false);
	}
      return true;
    }

    // Returns immediately
    bool sendPositionRequest(int pan, int tilt)
    {
      if (!writeCmd ("s ", buffer_))
      {
        ROS_ERROR ("Error entering slaved mode!");
        return (false);
      }
      char cmd[16];
      snprintf (cmd, 16, "pp%d ", pan);
      if (!writeCmd (cmd, buffer_))
      {
        ROS_ERROR ("Error setting pan position to: %d!", pan);
        return (false);
      }
      snprintf (cmd, 16, "tp%d ", tilt);
      if (!writeCmd (cmd, buffer_))
      {
        ROS_ERROR ("Error setting tilt position to: %d!", tilt);
        return (false);
      }
      if (!writeCmd ("i ", buffer_))
      {
        ROS_ERROR ("Error starting motion execution (entering immediate mode)!");
        return (false);
      }
      return true;
    }

    // Waits until a position is reached, and periodically publishes the current position
    void waitUntilPosition(int pan, int tilt, int tolerance)
    {
      while (nh_.ok ())
      {
	int current_pan  = getIntPosition ('p');
	int current_tilt = getIntPosition ('t');
	publishPosition(current_pan, current_tilt);
	if ((current_pan <= pan + tolerance) && (current_pan >= pan - tolerance)) {
	  if ((current_tilt <= tilt + tolerance) && (current_tilt >= tilt - tolerance)) {
	    break;
	  }
	}
	ros::Duration(0.01).sleep();
	//ros::spinOnce ();  // No need, we take over publishing the position
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Triggers a request via a service call */
    bool
      control2 (dp_ptu47_pan_tilt_stage::SendCommand::Request &req, dp_ptu47_pan_tilt_stage::SendCommand::Response &resp)
    {
      if (port_handle_ < 0)
        return (false);

      // Check limits
      if (!checkPositionRequest(req.pan_angle, req.tilt_angle))
      {
	return false;
      }

      // Get the RAW encoder count to move
      int pan = static_cast<int> (req.pan_angle / pan_resolution_);
      int tilt = static_cast<int> (req.tilt_angle / tilt_resolution_);

      if (!setupSmoothMotion())
      {
	return false;
      }

      // Command the motion
      if (!sendPositionRequest(pan, tilt))
      {
	return false;
      }

      // Should we monitor the motion until the desired position has been reached?
      if (req.wait_finished)
      {
	waitUntilPosition(pan, tilt, 0);
      }
      resp.pan_angle  = getPosition('p');
      resp.tilt_angle = getPosition('t');

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Triggers a request via a service call */
    bool
      controlTrajectory (dp_ptu47_pan_tilt_stage::SendTrajectory::Request &req, dp_ptu47_pan_tilt_stage::SendTrajectory::Response &resp)
    {
      if (port_handle_ < 0)
        return false;

      int tolerance = static_cast<int> (req.tolerance / ((tilt_resolution_ + pan_resolution_)/2));
      if (req.pan_angles.size() != req.tilt_angles.size()) {
	ROS_ERROR("Malformed pan-tilt trajectory");
	return false;
      }
      else
      {
	ROS_INFO("Received trajectory:");
	for (unsigned int i = 0; i < req.pan_angles.size(); i++) {
	  ROS_INFO("%f, %f", req.pan_angles[i], req.tilt_angles[i]);
	}
	ROS_INFO("Tolerance %f (%d)", req.tolerance, tolerance);
      }

      if (!setupSmoothMotion())
      {
	return false;
      }

      for (unsigned int i = 0; i < req.pan_angles.size(); i++)
      {
	// Check limits
	if (!checkPositionRequest(req.pan_angles[i], req.tilt_angles[i]))
	{
	  return false;
	}

	// Get the RAW encoder count to move
	int pan = static_cast<int> (req.pan_angles[i] / pan_resolution_);
	int tilt = static_cast<int> (req.tilt_angles[i] / tilt_resolution_);
	
	// Command the motion
	if (!sendPositionRequest(pan, tilt))
	{
	  return false;
	}

	// Should we monitor the motion until the desired position has been reached?
	if (i == req.pan_angles.size()-1)
	{
	  tolerance = 0;
	}
	waitUntilPosition(pan, tilt, tolerance);
      }
      resp.pan_angle  = getPosition('p');
      resp.tilt_angle = getPosition('t');

      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Dynamically reconfigure parameters */
    void
      config_callback (dp_ptu47_pan_tilt_stage::DpPtu47Config &config, uint32_t level)
    {
      if (!init_)
        return;
      // Enable / disable limits
      if (config.disable_limits != limits_disabled_)
      {
        limits_disabled_ = config.disable_limits;
        if (limits_disabled_)
        {
          ROS_INFO ("Disabling limits.");
          enableLimits ('d');
        }
        else
        {
          ROS_INFO ("Enabling limits.");
          enableLimits ('e');
        }
      }

      // Set the new limits
      if (config.limits_pan_min != user_min_pan_)
      {
        if (!limits_disabled_)
        {
          user_min_pan_ = config.limits_pan_min;
          if (setLimit ('p', 'n', user_min_pan_))
            ROS_INFO ("Setting new min pan limit to: %f", user_min_pan_);
        }
      }
      if (config.limits_pan_max != user_max_pan_)
      {
        if (!limits_disabled_)
        {
          user_max_pan_ = config.limits_pan_max;
          if (setLimit ('p', 'x', user_max_pan_))
            ROS_INFO ("Setting new max pan limit to: %f", user_max_pan_);
        }
      }
      if (config.limits_tilt_min != user_min_tilt_)
      {
        if (!limits_disabled_)
        {
          user_min_tilt_ = config.limits_tilt_min;
          if (setLimit ('t', 'n', user_min_tilt_))
            ROS_INFO ("Setting new min tilt limit to: %f", user_min_tilt_);
        }
      }
      if (config.limits_tilt_max != user_max_tilt_)
      {
        if (!limits_disabled_)
        {
          user_max_tilt_ = config.limits_tilt_max;
          if (setLimit ('t', 'x', user_max_tilt_))
            ROS_INFO ("Setting new max tilt limit to: %f", user_max_tilt_);
        }
      }

      // Read the actual values from the unit
      if (!limits_disabled_)
      {
        unit_min_pan_  = getLimit ('p', 'n');
        unit_max_pan_  = getLimit ('p', 'x');
        unit_min_tilt_ = getLimit ('t', 'n');
        unit_max_tilt_ = getLimit ('t', 'x');
        ROS_INFO ("Reconfigure request received. Limits enabled: %d, Pan limits: %f/%f, Tilt limits: %f/%f.",
                  !limits_disabled_, user_min_pan_, user_max_pan_, user_min_tilt_, user_max_tilt_);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      spin ()
    {
      while (nh_.ok ())
      {
        // Set slaved position execution mode
        if (!writeCmd ("s ", buffer_))
          ROS_ERROR ("Error setting unit into slave position execution mode!");

        float pan_angle  = getPosition ('p');
        float tilt_angle = getPosition ('t');

        // Publish a state stamped message with the current state (angle)
        dp_ptu47_pan_tilt_stage::PanTiltStamped pts;
        pts.pan_angle  = pan_angle;
        pts.tilt_angle = tilt_angle;
        pts.header.stamp = ros::Time::now ();
        pts.header.frame_id = "/ptu/base_link";
        stamped_pub_.publish (pts);

        ros::spinOnce ();
       }
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "dp_ptu47");

  DpPtu47 dp;
  // Enable the dynamic reconfigure service
  dynamic_reconfigure::Server<dp_ptu47_pan_tilt_stage::DpPtu47Config> srv;
  dynamic_reconfigure::Server<dp_ptu47_pan_tilt_stage::DpPtu47Config>::CallbackType f =  boost::bind (&DpPtu47::config_callback, &dp, _1, _2);
  srv.setCallback (f);

  if (!dp.init ())
    return (-1);

  dp.spin ();

  return (0);
}
/* ]--- */

