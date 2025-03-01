/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/transport/Node.hh>


ignition::transport::Node node;

// front
std::string topic_pub_front = "/us_front";   //publish to this topic
auto pub_front = node.Advertise<ignition::msgs::Double>(topic_pub_front);

// back
std::string topic_pub_back = "/us_back";   //publish to this topic
auto pub_back = node.Advertise<ignition::msgs::Double>(topic_pub_back);

// left
std::string topic_pub_left = "/us_left";   //publish to this topic
auto pub_left = node.Advertise<ignition::msgs::Double>(topic_pub_left);

// front
std::string topic_pub_right = "/us_right";   //publish to this topic
auto pub_right = node.Advertise<ignition::msgs::Double>(topic_pub_right);


//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb_front(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Double minRangeMsg;

  bool allMore = true;
  double minDistance = 1000;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < minDistance) 
    {
      minDistance = _msg.ranges(i);
    }
  }
  
  minRangeMsg.set_data(minDistance);

  pub_front.Publish(minRangeMsg);
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb_back(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Double minRangeMsg;

  bool allMore = true;
  double minDistance = 1000;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < minDistance) 
    {
      minDistance = _msg.ranges(i);
    }
  }
  
  minRangeMsg.set_data(minDistance);

  pub_back.Publish(minRangeMsg);
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb_left(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Double minRangeMsg;

  bool allMore = true;
  double minDistance = 1000;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < minDistance) 
    {
      minDistance = _msg.ranges(i);
    }
  }
  
  minRangeMsg.set_data(minDistance);

  pub_left.Publish(minRangeMsg);
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb_right(const ignition::msgs::LaserScan &_msg)
{
  ignition::msgs::Double minRangeMsg;

  bool allMore = true;
  double minDistance = 1000;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < minDistance) 
    {
      minDistance = _msg.ranges(i);
    }
  }
  
  minRangeMsg.set_data(minDistance);

  pub_right.Publish(minRangeMsg);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub_front = "/lidar_front";   // subscribe to this topic
  std::string topic_sub_back = "/lidar_back";
  std::string topic_sub_left = "/lidar_left";
  std::string topic_sub_right = "/lidar_right";
  std::string topic_pose = "/world/car_world/pose/info";

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub_front, cb_front))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub_front << "]" << std::endl;
    return -1;
  }

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub_back, cb_back))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub_back << "]" << std::endl;
    return -1;
  }

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub_left, cb_left))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub_left << "]" << std::endl;
    return -1;
  }

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub_right, cb_right))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub_right << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
