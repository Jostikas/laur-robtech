// Copyright (c) 2015-2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file mouse_driver.h
 *  On /mouse_driver/events topic it publishes mouse_driver::mouse_event
 *  messages that contain direction and integral of the turn wheel as well as the 
 *  information about push button being pressed or depressed.
 * 
 *  NOTE
 *  If you get permission denied when running this ROS node, use
 * 	ls -l /dev/input/event*
 *  to learn which group can access linux input events. Then add your username to
 *  that group by issuing
 *  	sudo usermod -a -G [group_name] [user_name]
 *  You need to log out and back in for these changes to take effect.
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include <linux/input.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <glob.h>	// for counting files in a dir (needed for counting event files in /dev/input)
#include <sstream>
#include <bitset>

#include "ros/ros.h"

#include "mouse_driver/mouse_event.h"

#ifndef MOUSE_DRIVER_H
#define MOUSE_DRIVER_H

class mouseDriver
{
public:

  // Constructor
  mouseDriver (std::string mouse_event_path)
  {
    if (mouse_event_path.empty() )
    {
      // Tries to find a suitable event file from the files available on the system
      descriptor_ = find_mouse();
    }
    else
    {
      // Tries to open user-specified event file as PowerMate device
      descriptor_ = open_mouse( mouse_event_path.c_str() );
    }
  };
  
  /** Goes through all the event files in /dev/input/ to locate a PowerMate device.
   *  @return file descriptor if all checks out; -1 otherwise.
   */
  int find_mouse();

  /** Opens the input device and checks whether its meaningful name (ie, EVIOCGNAME in ioctl()) contains substrings specified in valid_substring.
   *  @param device_path file path of a linux event.
   *  @return file descriptor if open and other checks have been succesfully passed; -1 otherwise.
   */
  int open_mouse(const char *device_path);

  /** Closes the device specificed by descriptor_. */
  void close_mouse();

  /** Checks if the PowerMate event file has been succesfully opened.
   *  @return TRUE when descriptor_ is not negative, FALSE otherwise.
   */
  bool isReadable();

  /** Searches the input event for type EV_KEY and EV_REL, publishes PowermateEvent message.
   *  @param ev input event.
   *  @param ros_publisher ROS publisher used to publish PowermateEvent message.
   */
  void processEvent(struct input_event *ev, ros::Publisher& ros_publisher);

  /** Reads linux event data, sends it to processing and publishing.
   *  @param ros_publisher ROS publisher used to publish PowermateEvent message.
   */
  void spin_mouse_driver(ros::Publisher& ros_publisher);

private:
  /** File descriptor of the working Griffin PowerMate event file. */
  int descriptor_;
  
  std::bitset<5> pressed_btn;
  
  /** A list of substrings that would indicate that a device is a mouse USB device. */
  std::vector<std::string> valid_substrings_ =
  {
    "mouse",
    "Mouse"
  };
 
}; // end class PowerMate

#endif
