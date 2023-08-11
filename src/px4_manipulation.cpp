/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_avoidance.cpp
 *
 * px4 manipulation
 *
 */

#include "px4_manipulation/px4_manipulation.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


Px4Manipulation::Px4Manipulation() : Node("minimal_publisher") {
    
	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Publishers
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
    
    // Subscribers
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos_profile, std::bind(&Px4Manipulation::vehicleStatusCallback, this, std::placeholders::_1));
    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_profile, std::bind(&Px4Manipulation::vehicleAttitudeCallback, this, std::placeholders::_1));
    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_profile, std::bind(&Px4Manipulation::vehicleLocalPositionCallback, this, std::placeholders::_1));
    // Setup loop timers
    statusloop_timer_ = this->create_wall_timer(50ms, std::bind(&Px4Manipulation::statusloopCallback, this));
}



void Px4Manipulation::statusloopCallback() {
    ///TODO: Get mission items
    // RCLCPP_INFO(this->get_logger(), "Publishing: %f", double(vehicle_nav_state_));

    ///TODO: Solve problem based on mission items using OMPL

    ///TODO: Get reference state from solution path

    ///TODO: Send reference state to vehicle

    // Publish offboard control mode
    px4_msgs::msg::OffboardControlMode offboard_ctrl_mode_msg;
    offboard_ctrl_mode_msg.position=true;
    offboard_ctrl_mode_msg.velocity=false;
    offboard_ctrl_mode_msg.acceleration=false;
    offboard_mode_pub_->publish(offboard_ctrl_mode_msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", offboard_ctrl_mode_msg.data.c_str());
}

void Px4Manipulation::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus &msg) {
    vehicle_nav_state_ = msg.nav_state;
    // RCLCPP_INFO(this->get_logger(), "Publishing: %f", double(vehicle_nav_state_));
}

void Px4Manipulation::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg) {
    ///TODO: Get vehicle attitude
    vehicle_attitude_(0) = msg.q[0];
    vehicle_attitude_(1) = msg.q[1];
    vehicle_attitude_(2) = -msg.q[2];
    vehicle_attitude_(3) = -msg.q[3];

}

void Px4Manipulation::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition &msg) {
    ///TODO: Get vehicle attitude
    vehicle_position_(0) = msg.x;
    vehicle_position_(1) = -msg.y;
    vehicle_position_(2) = -msg.z;
}
