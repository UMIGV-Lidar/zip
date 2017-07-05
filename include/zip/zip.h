#ifndef ZIP_H
#define ZIP_H

#include <cstddef>
#include <tuple>
#include <utility>
#include <iterator>
#include <functional>
#include <algorithm>

template <typename Tuple, std::size_t N>
struct tuple_any_equal_impl {
	static bool apply(const Tuple &lhs, const Tuple &rhs) {
		if (!(tuple_any_equal_impl<Tuple, N - 1>::apply(lhs, rhs))) {
			return std::get<N - 1>(lhs) == std::get<N - 1>(rhs);
		}

		return true;
	}
};

template <typename Tuple>
struct tuple_any_equal_impl<Tuple, 1> {
	static bool apply(const Tuple &lhs, const Tuple &rhs) {
		return std::get<0>(lhs) == std::get<0>(rhs);
	}
};

template <typename... Types>
static bool tuple_any_equal(
	const std::tuple<Types...> &lhs,
	const std::tuple<Types...> &rhs
) {
	using tuple_type = std::tuple<Types...>;
	return tuple_any_equal_impl<tuple_type, sizeof...(Types)>::apply(lhs, rhs);
}

template <typename Tuple, typename Function, std::size_t... I>
static auto tuple_for_each_impl(
	Tuple &tuple,
	Function f,
	std::index_sequence<I...>
) {
	auto result = { (f(std::get<I>(tuple)), 0)... };
	return result;
}

template <typename Function, typename... Types>
static void tuple_for_each(std::tuple<Types...> &tuple, Function f) {
	tuple_for_each_impl(tuple, f, std::index_sequence_for<Types...>());
}

template <typename... BidirectionalIterators>
class zip_range {
	template <typename Iterator>
	using value_type = decltype(*std::declval<Iterator>);

	template <typename Iterator>
	using reference = value_type<Iterator>&;

	using tuple_type = std::tuple<BidirectionalIterators...>;

	using value_tuple = std::tuple<reference<BidirectionalIterators>...>;

	tuple_type firsts_, lasts_;
	std::size_t size_;

	template <std::size_t... I>
	static std::size_t min_size(
		const tuple_type &firsts,
		const tuple_type &lasts,
		std::index_sequence<I...>
	) {
		return std::min(
			{ std::distance(std::get<I>(firsts), std::get<I>(lasts))... }
		);
	}

public:
	class iterator :
		public std::iterator<std::bidirectional_iterator_tag, value_tuple>
	{
		friend class zip_range;

		tuple_type base_;

		explicit iterator(const tuple_type &base) : base_(base) { }

		template <typename Tuple, std::size_t N>
		struct dereference_impl {
			static auto apply(const Tuple &x) {
				using current_type = decltype(std::get<N - 1>(x));
				using value_type = decltype(*std::declval<current_type>());

				std::tuple<value_type&> current(*std::get<N - 1>(x));

				return std::tuple_cat(
					dereference_impl<Tuple, N - 1>::apply(x),
					current
				);
			}
		};

		template <typename Tuple>
		struct dereference_impl<Tuple, 1> {
			static auto apply(const Tuple &x) {
				using current_type = decltype(std::get<0>(x));
				using value_type = decltype(*std::declval<current_type>());

				return std::tuple<value_type&>(*std::get<0>(x));
			}
		};

		static auto dereference(const tuple_type &iterators) {
			return dereference_impl<
				tuple_type,
				sizeof...(BidirectionalIterators)
			>::apply(iterators);
		}

	public:
		using value_type = value_tuple;

		// returns true if either iterator is the same
		// no iterator alignment required to detect the end
		friend bool operator==(const iterator &lhs, const iterator &rhs) {
			return tuple_any_equal(lhs.base_, rhs.base_);
		}

		friend bool operator!=(const iterator &lhs, const iterator &rhs) {
			return !(lhs == rhs);
		}

		iterator& operator++() {
			auto increment = [](auto &x) { return ++x; };

			tuple_for_each(base_, increment);

			return *this;
		}

		iterator operator++(int) {
			auto temp = *this;

			++(*this);

			return temp;
		}

		iterator& operator--() {
			auto decrement = [](auto &x) { return --x; };

			tuple_for_each(base_, decrement);

			return *this;
		}

		iterator operator--(int) {
			auto temp = *this;

			--(*this);

			return temp;
		}

		auto operator*() const {
			return dereference(base_);
		}
	};

	// no default constructor

	// initialize with two ranges
	zip_range(
		const tuple_type &firsts,
		const tuple_type &lasts
	) :
		firsts_(firsts),
		size_(min_size(
			firsts,
			lasts,
			std::index_sequence_for<BidirectionalIterators...>()
		))
	{
		iterator advanced(firsts_);
		std::advance(advanced, size_);
		lasts_ = advanced.base_;
	}

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

template <typename... Ranges>
auto zip(Ranges&&... ranges) {
	return zip_range<decltype(std::begin(std::forward<Ranges>(ranges)))...>(
		std::make_tuple(std::begin(std::forward<Ranges>(ranges))...),
		std::make_tuple(std::end(std::forward<Ranges>(ranges))...)
	);
}

#endif