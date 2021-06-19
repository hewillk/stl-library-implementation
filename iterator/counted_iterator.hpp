#pragma once

#include "ranges_util.hpp"

namespace ranges {

template <class I>
struct counted_iter_value_type {};
template <std::indirectly_readable I>
struct counted_iter_value_type<I> {
  using value_type = std::iter_value_t<I>;
};

template <class I>
struct counted_iter_concept {};
template <class I>
requires requires {
  typename I::iterator_concept;
}
struct counted_iter_concept<I> {
  using iterator_concept = typename I::iterator_concept;
};

template <class I>
struct counted_iter_category {};
template <class I>
requires requires {
  typename I::iterator_category;
}
struct counted_iter_category<I> {
  using iterator_category = typename I::iterator_category;
};

template <std::input_or_output_iterator I>
class counted_iterator : counted_iter_value_type<I>,
                         counted_iter_concept<I>,
                         counted_iter_category<I> {
 public:
  using iterator_type = I;
  using difference_type = std::iter_difference_t<I>;

  constexpr counted_iterator() = default;
  constexpr counted_iterator(I x, std::iter_difference_t<I> n)
    : current_(std::move(x)), length_(n) {
    assert(n >= 0);
  }

  template <class I2>
  requires std::convertible_to<const I2&, I> constexpr counted_iterator(
    const counted_iterator<I2>& x)
    : current_(x.current_), length_(x.length_) {}

  template <class I2>
  requires std::assignable_from<I&, const I2&> constexpr counted_iterator& operator=(
    const counted_iterator<I2>& x) {
    current_ = x.current_;
    length_ = x.length_;
    return *this;
  }

  constexpr const I& base() const& { return current_; }
  constexpr I base() && { return std::move(current_); }
  constexpr std::iter_difference_t<I> count() const noexcept { return length_; }
  constexpr decltype(auto) operator*() {
    assert(length_ > 0);
    return *current_;
  }
  constexpr decltype(auto) operator*() const requires dereferenceable<const I> {
    assert(length_ > 0);
    return *current_;
  }

  constexpr auto operator->() const noexcept requires std::contiguous_iterator<I> {
    return std::addressof(current_);
  }

  constexpr counted_iterator& operator++() {
    assert(length_ > 0);
    ++current_;
    --length_;
    return *this;
  }
  decltype(auto) operator++(int) {
    assert(length_ > 0);
    --length_;
    try {
      return current_++;
    } catch (...) {
      ++length_;
      throw;
    }
  }
  constexpr counted_iterator operator++(int) requires std::forward_iterator<I> {
    counted_iterator tmp = *this;
    ++*this;
    return tmp;
  }
  constexpr counted_iterator& operator--() requires std::bidirectional_iterator<I> {
    --current_;
    ++length_;
    return *this;
  }
  constexpr counted_iterator operator--(int) requires std::bidirectional_iterator<I> {
    counted_iterator tmp = *this;
    --*this;
    return tmp;
  }

  constexpr counted_iterator operator+(std::iter_difference_t<I> n) const
    requires std::random_access_iterator<I> {
    return counted_iterator(current_ + n, length_ - n);
  }
  friend constexpr counted_iterator operator+(
    std::iter_difference_t<I> n,
    const counted_iterator& x) requires std::random_access_iterator<I> {
    return x + n;
  }
  constexpr counted_iterator& operator+=(
    std::iter_difference_t<I> n) requires std::random_access_iterator<I> {
    assert(n <= length_);
    current_ += n;
    length_ -= n;
    return *this;
  }
  constexpr counted_iterator operator-(std::iter_difference_t<I> n) const
    requires std::random_access_iterator<I> {
    return counted_iterator(current_ - n, length_ + n);
  }
  template <std::common_with<I> I2>
  friend constexpr std::iter_difference_t<I2> operator-(
    const counted_iterator& x, const counted_iterator<I2>& y) {
    return y.length_ - x.length_;
  }
  friend constexpr std::iter_difference_t<I> operator-(
    const counted_iterator& x, std::default_sentinel_t) {
    return -x.length_;
  }
  friend constexpr std::iter_difference_t<I> operator-(
    std::default_sentinel_t, const counted_iterator& y) {
    return y.length_;
  }

  constexpr counted_iterator& operator-=(
    std::iter_difference_t<I> n) requires std::random_access_iterator<I> {
    assert(-n <= length_);
    current_ -= n;
    length_ += n;
    return *this;
  }

  constexpr decltype(auto) operator[](std::iter_difference_t<I> n) const
    requires std::random_access_iterator<I> {
    assert(n < length_);
    return current_[n];
  }

  template <std::common_with<I> I2>
  friend constexpr bool operator==(const counted_iterator& x, const counted_iterator<I2>& y) {
    return x.length_ == y.length_;
  }
  friend constexpr bool operator==(const counted_iterator& x, std::default_sentinel_t) {
    return x.length_ == 0;
  }

  template <std::common_with<I> I2>
  friend constexpr std::strong_ordering operator<=>(
    const counted_iterator& x, const counted_iterator<I2>& y) {
    return y.length_ <=> x.length_;
  }

  friend constexpr std::iter_rvalue_reference_t<I> iter_move(const counted_iterator& i) noexcept(
    noexcept(std::ranges::iter_move(i.current_))) requires std::input_iterator<I> {
    assert(i.length_ > 0);
    return std::ranges::iter_move(i.current_);
  }
  template <std::indirectly_swappable<I> I2>
  friend constexpr void
  iter_swap(const counted_iterator& x, const counted_iterator<I2>& y) noexcept(
    noexcept(std::ranges::iter_swap(x.current_, y.current_))) {
    assert(x.length_ > 0 && y.length_ > 0);
    std::ranges::iter_swap(x.current_, y.current_);
  }

 private:
  I current_ = I();
  std::iter_difference_t<I> length_ = 0;

  template <std::input_or_output_iterator I2>
  friend class counted_iterator;
};

}  // namespace ranges

template <class I>
struct std::incrementable_traits<ranges::counted_iterator<I>> {
  using difference = std::iter_difference_t<I>;
};

template <std::input_iterator I>
struct std::iterator_traits<ranges::counted_iterator<I>> : std::iterator_traits<I> {
  using pointer = void;
};