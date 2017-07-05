## Synopsis

`zip` is a header-only library which provides an interface to iterate through two or more ranges at the same time.

## Code Example

	#include "ros/ros.h"
	#include "ros/console.h"
	#include "zip/zip.h"

	#include <vector>
	#include <string>
	#include <utility>
	#include <tuple>

	int main(int argc, char *argv[]) {
		ros::init(argc, argv, "zip_test");
		ros::NodeHandle n;

		std::vector<int> a = {0, 1, 2, 3};
		std::string b = "hi friend";

		for (const auto &zipped : zip(a, b)) {
			auto &first = std::get<0>(zipped);
			auto &second = std::get<1>(zipped);

			ROS_INFO_STREAM("first = " << first << ", second = " << second);
			
			++first;
			++second;
		}

		for (const auto &zipped : zip(a, b)) {
			auto &first = std::get<0>(zipped);
			auto &second = std::get<1>(zipped);

			ROS_INFO_STREAM("first = " << first << ", second = " << second);
		}
	}

## Motivation

Python's `zip` function is very nice and lends itself to use in range-based for loops. This is a port of sorts.

## Installation

Run `catkin_make install` on the contents of this package, then `#include "zip/zip.h"` to make use of it.

## Contributors

`zip` is authored and maintained by Gregory Meyer of UMIGV.