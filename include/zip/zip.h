#ifndef ZIP_H
#define ZIP_H

#include "zip2.h"
#include "zip3.h"

#include <utility>
#include <iterator>

template <typename Range1, typename Range2>
auto zip(Range1 &&first, Range2 &&second) {
	auto begins = std::make_pair(
		std::begin(std::forward<Range1>(first)),
		std::begin(std::forward<Range2>(second))
	);

	auto ends = std::make_pair(
		std::end(std::forward<Range1>(first)),
		std::end(std::forward<Range2>(second))
	);

	return zip2_range<
		decltype(begins.first),
		decltype(begins.second)
	>(begins, ends);
}

#endif