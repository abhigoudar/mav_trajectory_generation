/*
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mav_visualization/hexacopter_marker.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node("hexacopter_publisher", options);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>
    ("marker_array", rclcpp::SystemDefaultsQoS());

  std::string frame_id("state");
  double scale = 1.0;
  bool simple = false;

  assert(false && " Hexacopter marker params not read");
  // nh_private.param("frame_id", frame_id, frame_id);
  // nh_private.param("scale", scale, scale);
  // nh_private.param("simple", simple, simple);

  mav_visualization::HexacopterMarker hex(simple);
  visualization_msgs::msg::MarkerArray markers;

  hex.setLifetime(0.0);
  hex.setAction(visualization_msgs::msg::Marker::ADD);

  std_msgs::msg::Header header;
  header.frame_id = frame_id;

  while (rclcpp::ok()) {
    header.stamp = this->get_clock()->now();
    hex.setHeader(header);
    hex.getMarkers(markers, scale, false);
    marker_pub->publish(markers);
    rclcpp::sleep_for(50*1e9);
  }
}
