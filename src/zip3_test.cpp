#include "ros/ros.h"
#include "ros/console.h"
#include "zip/zip.h"

#include <vector>
#include <string>
#include <utility>
#include <tuple>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "zip2_test");
	ros::NodeHandle n;

	std::vector<int> a = {0, 1, 2, 3};
	std::string b = "hi friend";
	std::vector<double> c = {10, 9, 8};

	for (const auto &zipped : zip(a, b, c)) {
		auto &first = std::get<0>(zipped);
		auto &second = std::get<1>(zipped);
		auto &third = std::get<2>(zipped);

		ROS_INFO_STREAM("first = " << first << ", second = " << second << ", third = " << third);
		
		++first;
		++second;
		++third;
	}

	for (const auto &zipped : zip(a, b, c)) {
		auto &first = std::get<0>(zipped);
		auto &second = std::get<1>(zipped);
		auto &third = std::get<2>(zipped);

		ROS_INFO_STREAM("first = " << first << ", second = " << second << ", third = " << third);
	}
}