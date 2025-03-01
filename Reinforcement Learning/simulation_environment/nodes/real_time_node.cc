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
#include <ignition/msgs/world_stats.pb.h>
#include <ignition/transport/Node.hh>

ignition::transport::Node node;

// front
std::string topic_rtf = "/factor";   //publish to this topic
auto pub_rtf = node.Advertise<ignition::msgs::Double>(topic_rtf);

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb_rtf(const ignition::msgs::WorldStatistics &_msg)
{
  ignition::msgs::Double percentMsg;
  
  percentMsg.set_data(_msg.real_time_factor());

  pub_rtf.Publish(percentMsg);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub_rtf = "/world/car_world/stats";

  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub_rtf, cb_rtf))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub_rtf << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}