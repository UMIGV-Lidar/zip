#include "ros/ros.h"
#include "ros/console.h"
#include "zip/zip.h"

#include <vector>
#include <string>
#include <utility>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "zip2_test");
	ros::NodeHandle n;

	std::vector<int> a = {0, 1, 2, 3};
	std::string b = "hi friend";

	for (const auto &zipped : zip(a, b)) {
		auto &first = zipped.first;
		auto &second = zipped.second;

		ROS_INFO_STREAM("first = " << first ", second = " << second);
	}
}