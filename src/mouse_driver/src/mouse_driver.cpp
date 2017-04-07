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

#include "mouse_driver/mouse_driver.h"

/** Opens the input device and checks whether its meaningful name (ie, EVIOCGNAME in ioctl()) is listed in valid_substrings_.
 *  @param device_path file name for linux event.
 *  @return file descriptor to mouse_driver event if all checks out, otherwise -1.
 */
int mouseDriver::open_mouse(const char *device_path)
{
  printf("Opening device: %s \n", device_path);
  
  // Open device at device_path for READONLY and get file descriptor 
  int fd = open(device_path, O_RDONLY);
  
  // If failed to open device at device_path
  if(fd < 0)
  {
    ROS_ERROR("Failed to open \"%s\"\n", device_path);
    return -1;
  }

  // Meaningful, i.e., EVIOCGNAME name
  char name[255];
  // Fetch the meaningful (i.e., EVIOCGNAME) name
  if(ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0)
  {
    ROS_ERROR("\"%s\": EVIOCGNAME failed.", device_path);
    close(fd);
    // Returns -1 if failed to fetch the meaningful name
    return -1;
  }

  // Let's check if the meaningful name matches one listed in valid_substrings_
  std::ostringstream sstream;
  // Convert name given as char* to stringstream
  sstream << name;
  // stringstream to string
  std::string name_as_string = sstream.str();
  int i;
  for (i=0; i < valid_substrings_.size(); i++)
  {
    // Does the meaningful name contain a predefined valid substring?
    std::size_t found = name_as_string.find( valid_substrings_[i] );
    if (found!=std::string::npos)
    {
      // if everything checks out, print on screen and return the file descriptor
      ROS_INFO("Found \x1b[1;34m'%s'\x1b[0m device. Starting to read ...\n", name);
      return fd;
    } // end if
  } // end for
  
  close(fd);
  return -1;
} // end openmouse_driver

/** Goes through all the event files in /dev/input/ to locate mouse_driver USB.
 *  @return file descriptor if all checks out, otherwise -1.
 */
int mouseDriver::find_mouse()
{
  // Using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  // Number of event files found in /dev/input/
  int num_event_dev = 0;
  // Counts for filenames that match the given pattern
  if(glob("/dev/input/event*", GLOB_NOSORT, NULL, &gl) == 0)
  {
    // Get number of event files
    num_event_dev = gl.gl_pathc;
  }

  int i, r;
  // Goes through all the found event files
  for(i = 0; i < num_event_dev; i++)
  {
    // Tries to open an event file as a mouse_driver device
    r = open_mouse(gl.gl_pathv[i]);
    // If opened file is mouse_driver event, return file descriptor
    if(r >= 0) return r;
  } // for
  
  // free memory allocated for globe struct
  globfree(&gl);

  // return error -1 because no mouse_driver device was found
  return -1;
} // end findmouse_driver

/** Closes the device specificed by descriptor_. */
void mouseDriver::close_mouse()
{
  printf("Closing mouse_driver device.\n");
  close(descriptor_);
  return;
}

/** Checks if the mouse_driver event file has been succesfully opened.
 *  @return TRUE when descriptor_ is not negative, FALSE otherwise.
 */
bool mouseDriver::isReadable ()
{
  if (descriptor_ < 0) return false;
  return true;
}

/** Processes the event data and publishes it as mouse_event message.
 *  @param ev input event.
 *  @param ros_publisher ROS publisher.
 */
void mouseDriver::processEvent(struct input_event *ev, ros::Publisher& ros_publisher)
{
  // mouse_event ROS message
  mouse_driver::mouse_event ros_message;

  // Supported events:
  //   Event type 0 (EV_SYN)
  //   Event type 1 (EV_KEY)
  //    Event code 272 (BTN_LEFT)
  //    Event code 273 (BTN_RIGHT)
  //    Event code 274 (BTN_MIDDLE)
  //    Event code 275 (BTN_SIDE)
  //    Event code 276 (BTN_EXTRA)
  //  Event type 2 (EV_REL)
  //    Event code 0 (REL_X)
  //    Event code 1 (REL_Y)
  //    Event code 6 (REL_HWHEEL)
  //    Event code 8 (REL_WHEEL)
  //  Event type 4 (EV_MSC)
  //    Event code 4 (MSC_SCAN)
  // -------------------------------------------------------------------
  
  // Switch to a case based on the event type
  switch(ev->type)
  {
    case EV_SYN:				// no need to do anything
//      printf("SYN REPORT\n");
      break; 
    case EV_MSC:				// unused for this ROS publisher
//      ROS_INFO("The LED pulse settings were changed; code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      break;
    case EV_REL:				// Upon receiving movement data
      switch(ev->code)
      {
        case REL_X:
          ros_message.X_btnchange = (int16_t)ev->value;
          ros_message.Y_btnstate = 0;
          ros_message.type = 1;
          break;
        case REL_Y:
          ros_message.X_btnchange = 0;
          ros_message.Y_btnstate = (int16_t)ev->value;
          ros_message.type = 1;
          break;
        case REL_WHEEL:
          ros_message.X_btnchange = 0;
          ros_message.Y_btnstate = (int16_t)ev->value;
          ros_message.type = 2;
          break;
        case REL_HWHEEL:
          ros_message.X_btnchange = (int16_t)ev->value;
          ros_message.Y_btnstate = 0;
          ros_message.type = 2;
          break;
        }
        //Publish the message
        ros_publisher.publish(ros_message);
      break;
    case EV_KEY:				// Upon receiving data about pressing and depressing the dial button
      switch(ev->code)
      {
          size_t bitpos;
        case BTN_LEFT:
          bitpos = 0;
          ros_message.X_btnchange = (int16_t)(1<<bitpos);
          ros_message.Y_btnstate = pressed_btn.set(bitpos, ev->value).to_ulong();
          ros_message.type = 0;
          break;
        case BTN_RIGHT:
          bitpos = 1;
          ros_message.X_btnchange = (int16_t)(1<<bitpos);
          ros_message.Y_btnstate = pressed_btn.set(bitpos, ev->value).to_ulong();
          ros_message.type = 0;
          break;
        case BTN_MIDDLE:
          bitpos = 2;
          ros_message.X_btnchange = (int16_t)(1<<bitpos);
          ros_message.Y_btnstate = pressed_btn.set(bitpos, ev->value).to_ulong();
          ros_message.type = 0;
          break;
        case BTN_SIDE:
          bitpos = 3;
          ros_message.X_btnchange = (int16_t)(1<<bitpos);
          ros_message.Y_btnstate = pressed_btn.set(bitpos, ev->value).to_ulong();
          ros_message.type = 0;
          break;
        case BTN_EXTRA:
          bitpos = 4;
          ros_message.X_btnchange = (int16_t)(1<<bitpos);
          ros_message.Y_btnstate = pressed_btn.set(bitpos, ev->value).to_ulong();
          ros_message.type = 0;
        }
        //Publish the message
        ros_publisher.publish(ros_message);
        break;
    default:					// default case
      ROS_WARN("Unexpected event type; ev->type = 0x%04x\n", ev->type);
  } // end switch

  fflush(stdout);
} // end processEvent

/** Method for reading the event data and ROS spinning.
 *  @param ros_publisher ROS publisher used to publish mouse_event message.
 */
void mouseDriver::spin_mouse_driver(ros::Publisher& ros_publisher)
{
  int const BUFFER_SIZE = 32;
  
  // see: https://www.kernel.org/doc/Documentation/input/input.txt
  struct input_event ibuffer[BUFFER_SIZE];
  int r, events, i;

  while( ros::ok() )
  {  
    // read() reads a binary file [http://pubs.opengroup.org/onlinepubs/9699919799/functions/read.html] and returns the number of bytes read.
    // The program waits in read() until there's something to read; thus it always gets a new event but ROS cannot make a clean exit while in read().
    // TODO: Figure out a way for ROS to exit cleanly.
    r = read(descriptor_, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
    if( r > 0 )
    {
      // Calculate the number of events
      events = r / sizeof(struct input_event);
      // Go through each read events
      for(i = 0; i < events; i++)
      {
	// Process event and publish data
	processEvent(&ibuffer[i], ros_publisher);
	// spin
	ros::spinOnce();
      } // end for
    } // end if
    else
    {
      // Let user know if read() has failed
      ROS_WARN("read() failed.\n");
      return;
    } // end else

  } // end while
  
  return;
} // end spin_mouse_driver

/** Main method. */
int main(int argc, char *argv[])
{
  // ROS init
  ros::init(argc, argv, "mouse_driver");
  
  // Private nodehandle for ROS
  ros::NodeHandle pnh("~");
  
  // Getting user-specified path from ROS parameter server
  std::string mouse_driver_path;
  pnh.param<std::string>("path", mouse_driver_path, "");
  
  // Let's construct mouse_driver object 
  mouseDriver the_driver(mouse_driver_path);
  
  // If failed to open any mouse_driver USB device, print info and exit
  if( !the_driver.isReadable() )
  {
    ROS_ERROR("Unable to locate any mouse_driver device.");
    ROS_INFO("You may try specifying path as ROS parameter, e.g., rosrun mouse_driver mouse_driver _path:=<device_event_path>");
    return -1;
  }

  // Creates publisher that advertises mouse_driver::mouse_event messages on topic /mouse_driver/events
  ros::Publisher pub_mouse_events = pnh.advertise<mouse_driver::mouse_event>("events", 100);
  
  // After mouse_driver is succesfully opened, read its input, publish ROS messages, and spin.
  the_driver.spin_mouse_driver(pub_mouse_events);

  // Close mouse_driver
  the_driver.close_mouse();

  return 0;
} //end main
