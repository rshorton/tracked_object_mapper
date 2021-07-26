/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "robot_head_interfaces/msg/track_status.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;

class MapTrackedObjectToMap : public rclcpp::Node
{
public:
	MapTrackedObjectToMap(const std::string & name)
  	  : Node(name),
		tfBuffer_(this->get_clock()),
		x_ave_(0.0),
		y_ave_(0.0),
		z_ave_(0.0),
		detect_cnt_(0),
		min_det_thresh_(3)
  	{
		subTrackedObject_ = create_subscription<robot_head_interfaces::msg::TrackStatus>(
	      "/head/tracked", 10, std::bind(&MapTrackedObjectToMap::tracked_object, this, _1));

		pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tracked_object_map_position", 10);

		tfl_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);

		last_ = std::chrono::high_resolution_clock::now();
  	}

private:
	void callback_clicked(
			const geometry_msgs::msg::PointStamped::SharedPtr msg) const {
		geometry_msgs::msg::PoseStamped pose;
		pose.header = msg->header;
		pose.pose.position = msg->point;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		pub_->publish(pose);
	}

	void tracked_object(const robot_head_interfaces::msg::TrackStatus::SharedPtr trk)
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed = now - last_;

		typedef std::chrono::duration<float> float_seconds;
		auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

		if (trk->tracking) {
			geometry_msgs::msg::TransformStamped transformStamped;
			try{
				transformStamped = tfBuffer_.lookupTransform("map", "oakd", rclcpp::Time(0));

				trk->object.x /= 1000.0;
				trk->object.y /= 1000.0;
				trk->object.z /= 1000.0;

				x_ave_ = x_ave_*0.7 + trk->object.x*0.3;
				y_ave_ = y_ave_*0.7 + trk->object.y*0.3;
				z_ave_ = z_ave_*0.7 + trk->object.z*0.3;

				//RCLCPP_INFO(this->get_logger(), "Obj x,y: %f, %f", trk->object.x, trk->object.y);
				if (seconds.count() >= 1.0) {

					geometry_msgs::msg::PointStamped pt;
					pt.point.x = x_ave_;
					pt.point.y = y_ave_;
					pt.point.z = z_ave_;
					pt.header.stamp = this->now();
					pt.header.frame_id = "oakd";

					geometry_msgs::msg::PointStamped transformed_pt;
					tf2::doTransform(pt, transformed_pt, transformStamped);

					geometry_msgs::msg::PoseStamped msgPose;
					msgPose.header.frame_id = "map";
					msgPose.header.stamp = this->now();
					msgPose.pose.position.x = transformed_pt.point.x;
					msgPose.pose.position.y = transformed_pt.point.y;
					msgPose.pose.position.z = 0.0;
					msgPose.pose.orientation.x = 0;
					msgPose.pose.orientation.y = 0;
					msgPose.pose.orientation.z = 0;
					msgPose.pose.orientation.w = 1;

					pub_->publish(msgPose);

					RCLCPP_INFO(this->get_logger(), "Obj: id: %d x,y: %f, %f,   pose x,y: %f, %f",
													trk->object.id, trk->object.x, trk->object.y,
													msgPose.pose.position.x, msgPose.pose.position.y);
					last_ = now;
				}
			}
			catch (tf2::TransformException &ex) {
				RCLCPP_ERROR(this->get_logger(), "Failed to transform from oakd to map");
			}
		}
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

	rclcpp::Subscription<robot_head_interfaces::msg::TrackStatus>::SharedPtr subTrackedObject_;

	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	tf2_ros::Buffer tfBuffer_;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_;

	float x_ave_;
	float y_ave_;
	float z_ave_;

	int detect_cnt_;
	int min_det_thresh_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto map_tracked_object_node = std::make_shared<MapTrackedObjectToMap>("map_tracked_object");

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(map_tracked_object_node);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}
