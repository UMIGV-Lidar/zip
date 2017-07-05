#ifndef ZIP2_H
#define ZIP2_H

#include <cstddef>
#include <tuple>
#include <utility>
#include <iterator>
#include <functional>

template <typename BidirectionalIterator1, typename BidirectionalIterator2>
class zip2_range {
	using pair_type =
		std::pair<BidirectionalIterator1, BidirectionalIterator2>;

	using value_pair =
		std::pair<
			decltype(*std::declval<BidirectionalIterator1>())&,
			decltype(*std::declval<BidirectionalIterator2>())&
		>;

	pair_type firsts_, lasts_;
	std::size_t size_;

	static std::size_t min_size(
		const pair_type &firsts,
		const pair_type &lasts
	) {
		std::size_t first_size, second_size;

		first_size = std::distance(firsts.first, lasts.first);
		second_size = std::distance(firsts.second, lasts.second);

		return std::min(first_size, second_size);
	}

public:
	class iterator :
		public std::iterator<std::bidirectional_iterator_tag, value_pair>
	{
		pair_type base_;
		value_pair deref_base_;

	public:
		using value_type = value_pair;

		// returns true if either iterator is the same
		// no iterator alignment required to detect the end
		friend bool operator==(const iterator &lhs, const iterator &rhs) {
			return
				(lhs.base_.first == rhs.base_.first) ||
				(lhs.base_.second == rhs.base_.second);
		}

		friend bool operator!=(const iterator &lhs, const iterator &rhs) {
			return !(lhs == rhs);
		}

		iterator& operator++() {
			++base_.first;
			++base_.second;

			return *this;
		}

		iterator operator++(int) {
			auto temp = *this;

			++(*this);

			return temp;
		}

		iterator& operator--() {
			--base_.first;
			--base_.second;

			return *this;
		}

		iterator operator--(int) {
			auto temp = *this;

			--(*this);

			return temp;
		}

		const value_type& operator*() const {
			deref_base_ = std::make_pair(
				std::ref(*base_.first),
				std::ref(*base_.second)
			);

			return deref_base_;
		}

		const value_type* operator->() const {
			return &deref_base_;
		}
	};

	// no default constructor

	// initialize with two ranges
	zip2_range(
		pair_type &&firsts,
		pair_type &&lasts
	) :
		firsts_(std::forward<pair_type>(firsts)),
		lasts_(std::forward<pair_type>(lasts)),
		size_(min_size(
			std::forward<pair_type>(firsts),
			std::forward<pair_type>(lasts)
		))
	{ }

	std::size_t size() const {
		return size_;
	}

	iterator begin() {
		return iterator(firsts_);
	}

	iterator end() {
		return iterator(lasts_);
	}
};

#endif