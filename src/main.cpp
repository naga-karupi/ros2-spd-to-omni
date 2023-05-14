#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <chrono>
#include <array>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Dense>


class transform_node : public rclcpp::Node {
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;

	void sub(const geometry_msgs::msg::Twist::SharedPtr msg) {
		std_msgs::msg::Float32MultiArray pub_msg;
		
		Eigen::Vector3d velocity;
		velocity << msg->linear.x, msg->linear.y, msg->angular.z;

		Eigen::Matrix<double, 4, 3> converter;

		constexpr double tmp_val = 1/1.414213562373095048801688724209698078569671875376948073176679737990732478462107038850387534327641572735013846230912;
		constexpr double r       = 0.38;
		
		converter << -tmp_val,  tmp_val,-r,
					  tmp_val,  tmp_val,-r,
					 -tmp_val,  tmp_val, r,
					  tmp_val,  tmp_val, r;
					

		Eigen::Vector4d spd = converter * velocity;

		for(int i = 0; i < spd.size(); i++) {
			pub_msg.data.push_back(spd(i));
		}
		
		///need to change
		///todo

		pub->publish(pub_msg);
	}

public:
	transform_node(std::string node_name = "spd_to_omni"): Node(node_name) {
		twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
			"tf_spd_msg", 10, std::bind(&transform_node::sub, this, std::placeholders::_1));
		
		pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("omni_msg", 10);
	}
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<transform_node>());
	rclcpp::shutdown();
	return 0;
}
