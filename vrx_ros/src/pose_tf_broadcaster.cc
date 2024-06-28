/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  bool previous_tf_is_known;
  nav_msgs::msg::Odometry odom_; // odometry data in previous timestamp

  public: FramePublisher() : Node("frame_publisher")
  {
    this->declare_parameter<std::string>("world_frame", "");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<std::string>("model_name", "");    

    this->tfBroadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->tfBroadcasterStatic =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    odomPub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    previous_tf_is_known = false;

    this->subscription =
        this->create_subscription<tf2_msgs::msg::TFMessage>(
        "pose", 100,
        std::bind(&FramePublisher::HandlePose, this, _1));
    this->subscriptionStatic =
        this->create_subscription<tf2_msgs::msg::TFMessage>(
        "pose_static", 100,
        std::bind(&FramePublisher::HandlePoseStatic, this, _1));
  }

  private: void HandlePose(const std::shared_ptr<tf2_msgs::msg::TFMessage> _msg)
  {
    std::string world_frame, base_frame, model_name;
    this->get_parameter("world_frame", world_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("model_name", model_name);

    if (!world_frame.empty())
    {
      std::vector<geometry_msgs::msg::TransformStamped> filtered;
      for (auto transform: _msg->transforms)
      {
        if (transform.header.frame_id != world_frame)
        {
          if (transform.child_frame_id == model_name){
            transform.header.frame_id = world_frame;
            transform.child_frame_id = base_frame;

            // compute and publish odometry data
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header = transform.header;
            odom_msg.child_frame_id = transform.child_frame_id;
            odom_msg.pose.pose.position.x = transform.transform.translation.x;
            odom_msg.pose.pose.position.y = transform.transform.translation.y;
            odom_msg.pose.pose.position.z = transform.transform.translation.z;
            odom_msg.pose.pose.orientation = transform.transform.rotation;
            if (previous_tf_is_known){
              long int nano  = (int) transform.header.stamp.nanosec;
              long int nano_ = (int) odom_.header.stamp.nanosec;
              double deltaT = ((double) (transform.header.stamp.sec - odom_.header.stamp.sec)) +
                 ((double) (nano - nano_))*1e-9;
              if (deltaT > 0.0){
                odom_msg.twist.twist.linear.x = (odom_msg.pose.pose.position.x - odom_.pose.pose.position.x) / deltaT;
                odom_msg.twist.twist.linear.y = (odom_msg.pose.pose.position.y - odom_.pose.pose.position.y) / deltaT;
                odom_msg.twist.twist.linear.z = (odom_msg.pose.pose.position.z - odom_.pose.pose.position.z) / deltaT;

                // equations derived at https://mariogc.com/post/angular-velocity-quaternions/
                odom_msg.twist.twist.angular.x = 2.0/deltaT*(
                  odom_.pose.pose.orientation.w * odom_msg.pose.pose.orientation.x -
                  odom_.pose.pose.orientation.x * odom_msg.pose.pose.orientation.w -
                  odom_.pose.pose.orientation.y * odom_msg.pose.pose.orientation.z +
                  odom_.pose.pose.orientation.z * odom_msg.pose.pose.orientation.y
                  );
                odom_msg.twist.twist.angular.y = 2.0/deltaT*(
                  odom_.pose.pose.orientation.w * odom_msg.pose.pose.orientation.y +
                  odom_.pose.pose.orientation.x * odom_msg.pose.pose.orientation.z -
                  odom_.pose.pose.orientation.y * odom_msg.pose.pose.orientation.w -
                  odom_.pose.pose.orientation.z * odom_msg.pose.pose.orientation.x
                  );
                odom_msg.twist.twist.angular.z = 2.0/deltaT*(
                  odom_.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z -
                  odom_.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y +
                  odom_.pose.pose.orientation.y * odom_msg.pose.pose.orientation.x -
                  odom_.pose.pose.orientation.z * odom_msg.pose.pose.orientation.w
                  );
              }
              else{
                odom_msg.twist.twist.linear.x = odom_.twist.twist.linear.x;
                odom_msg.twist.twist.linear.y = odom_.twist.twist.linear.y;
                odom_msg.twist.twist.linear.z = odom_.twist.twist.linear.z;
                odom_msg.twist.twist.angular.x = odom_.twist.twist.angular.x;
                odom_msg.twist.twist.angular.y = odom_.twist.twist.angular.y;
                odom_msg.twist.twist.angular.z = odom_.twist.twist.angular.z;
              }
            }
            else{
              odom_msg.twist.twist.linear.x = 0.0;
              odom_msg.twist.twist.linear.y = 0.0;
              odom_msg.twist.twist.linear.z = 0.0;
              odom_msg.twist.twist.angular.x = 0.0;
              odom_msg.twist.twist.angular.y = 0.0;
              odom_msg.twist.twist.angular.z = 0.0;
            }
            odomPub->publish(odom_msg);
            odom_ = odom_msg;
            if (!previous_tf_is_known) previous_tf_is_known = true;
          }
          filtered.push_back(transform);
        }
      }
      this->tfBroadcaster->sendTransform(filtered);
    }
    else
    {
      // Send the transformation
      this->tfBroadcaster->sendTransform(_msg->transforms);
    }
  }

  private: void HandlePoseStatic(const std::shared_ptr<tf2_msgs::msg::TFMessage> _msg)
  {
    std::string world_frame, base_frame, model_name;
    this->get_parameter("world_frame", world_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("model_name", model_name);
    if (!world_frame.empty())
    {
      std::vector<geometry_msgs::msg::TransformStamped> filtered;
      for (auto transform: _msg->transforms)
      {
        if (transform.header.frame_id != world_frame)
        {
          if (transform.child_frame_id == model_name){
            transform.header.frame_id = world_frame;
            transform.child_frame_id = base_frame;
          }
          filtered.push_back(transform);
        }
      }
      this->tfBroadcasterStatic->sendTransform(filtered);
    }
    else
    {
      // Send the transformation
      this->tfBroadcasterStatic->sendTransform(_msg->transforms);
    }
  }

  private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription;
  private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
               subscriptionStatic;
  private: std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  private: std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
               tfBroadcasterStatic;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
