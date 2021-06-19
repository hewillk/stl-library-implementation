#pragma once

#include "ref_view.hpp"

namespace ranges {

template <class T, std::size_t N>
concept has_tuple_element = requires(T t) {
  typename std::tuple_size<T>::type;
  requires N < std::tuple_size_v<T>;
  typename std::tuple_element_t<N, T>;
  { std::get<N>(t) }
  ->std::convertible_to<const std::tuple_element_t<N, T>&>;
};

template <class T, std::size_t N>
concept returnable_element =
  std::is_reference_v<T> || std::move_constructible<std::tuple_element_t<N, T>>;

template <std::ranges::input_range V, std::size_t N>
requires std::ranges::view<V>&& has_tuple_element<std::ranges::range_value_t<V>, N>&&
  has_tuple_element<std::remove_reference_t<std::ranges::range_reference_t<V>>, N>&&
    returnable_element<std::ranges::range_reference_t<V>, N> 
 class elements_view
  : public view_interface<elements_view<V, N>> {
 private:
  template <class Base>
  struct iter_cat {};

  template <std::ranges::forward_range Base>
  struct iter_cat<Base> {
   private:
    static auto iter_category() {
      using C = std::iterator_traits<std::ranges::iterator_t<Base>>::iterator_category;
      using R = decltype(std::get<N>(*std::declval<std::ranges::iterator_t<Base>>()));
      if constexpr (!std::is_lvalue_reference_v<R>)
        return std::input_iterator_tag{};
      else if constexpr (std::derived_from<C, std::random_access_iterator_tag>)
        return std::random_access_iterator_tag{};
      else
        return C{};
    }

   public:
    using iterator_category = decltype(iter_category());
  };

  template <bool Const>
  struct sentinel;

  template <bool Const>
  struct iterator : iter_cat<maybe_const<Const, V>> {
   private:
    using Base = maybe_const<Const, V>;

    std::ranges::iterator_t<Base> current_ = std::ranges::iterator_t<Base>();

    static constexpr decltype(auto) get_element(const std::ranges::iterator_t<Base>& i) {
      if constexpr (std::is_reference_v<std::ranges::range_reference_t<Base>>)
        return std::get<N>(*i);
      else {
        using E = std::remove_cv_t<std::tuple_element_t<N, std::ranges::range_reference_t<Base>>>;
        return static_cast<E>(std::get<N>(*i));
      }
    }

    static auto iter_concept() {
      if constexpr (std::ranges::random_access_range<V>)
        return std::random_access_iterator_tag{};
      else if constexpr (std::ranges::bidirectional_range<V>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (std::ranges::forward_range<V>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

   public:
    using iterator_concept = decltype(iter_concept());
    using value_type =
      std::remove_cvref_t<std::tuple_element_t<N, std::ranges::range_value_t<Base>>>;
    using difference_type = std::ranges::range_difference_t<Base>;

    iterator() = default;
    constexpr explicit iterator(std::ranges::iterator_t<Base> current)
      : current_(std::move(current)) {}
    constexpr iterator(iterator<!Const> i) requires Const&& std::convertible_to<
      std::ranges::iterator_t<V>, std::ranges::iterator_t<Base>> : current_(std::move(i.current_)) {
    }

    constexpr std::ranges::iterator_t<Base> base()
      const& requires std::copyable<std::ranges::iterator_t<Base>> {
      return current_;
    }
    constexpr std::ranges::iterator_t<Base> base() && { return std::move(current_); }

    constexpr decltype(auto) operator*() const { return get_element(current_); }

    constexpr iterator& operator++() {
      ++current_;
      return *this;
    }
    constexpr void operator++(int) { ++current_; }
    constexpr iterator operator++(int) requires std::ranges::forward_range<Base> {
      auto tmp = *this;
      ++current_;
      return tmp;
    }

    constexpr iterator& operator--() requires std::ranges::bidirectional_range<Base> {
      --current_;
      return *this;
    }
    constexpr iterator operator--(int) requires std::ranges::bidirectional_range<Base> {
      auto tmp = *this;
      --current_;
      return tmp;
    }

    constexpr iterator& operator+=(
      difference_type n) requires std::ranges::random_access_range<Base> {
      current_ += n;
      return *this;
    }
    constexpr iterator& operator-=(
      difference_type n) requires std::ranges::random_access_range<Base> {
      current_ -= n;
      return *this;
    }

    constexpr decltype(auto) operator[](difference_type n) const
      requires std::ranges::random_access_range<Base> {
      return get_element(current_ + n);
    }

    friend constexpr bool operator==(
      const iterator& x,
      const iterator& y) requires std::equality_comparable<std::ranges::iterator_t<Base>> {
      return x.current_ == y.current_;
    }

    friend constexpr bool operator<(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return x.current_ < y.current_;
    }
    friend constexpr bool operator>(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return y < x;
    }
    friend constexpr bool operator<=(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return !(y < x);
    }
    friend constexpr bool operator>=(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return !(x < y);
    }
    friend constexpr bool operator<=>(const iterator& x, const iterator& y) requires std::ranges::
      random_access_range<Base>&& std::three_way_comparable<std::ranges::iterator_t<Base>> {
      return x.current_ <=> y.current_;
    }

    friend constexpr iterator operator+(
      const iterator& x, difference_type y) requires std::ranges::random_access_range<Base> {
      return iterator{x} += y;
    }
    friend constexpr iterator operator+(
      difference_type x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return y + x;
    }
    friend constexpr bool operator-(
      const iterator& x, difference_type y) requires std::ranges::random_access_range<Base> {
      return iterator{x} -= y;
    }
    friend constexpr difference_type operator-(const iterator& x, const iterator& y) requires std::
      sized_sentinel_for<std::ranges::iterator_t<Base>, std::ranges::iterator_t<Base>> {
      return x.current_ - y.current_;
    }

    friend iterator<!Const>;
    template <bool>
    friend class sentinel;
  };

  template <bool Const>
  struct sentinel {
   private:
    template <bool Const2>
    constexpr bool equal(const iterator<Const2>& x) const {
      return x.current_ == end_;
    }
    template <bool Const2>
    constexpr auto distance_from(const iterator<Const2>& i) const {
      return end_ - i.current_;
    }

    using Base = maybe_const<Const, V>;
    std::ranges::sentinel_t<Base> end_ = std::ranges::sentinel_t<Base>();

   public:
    sentinel() = default;
    constexpr explicit sentinel(std::ranges::sentinel_t<Base> end) : end_(end) {}
    constexpr sentinel(sentinel<!Const> other) requires Const&& std::convertible_to<
      std::ranges::sentinel_t<V>, std::ranges::sentinel_t<Base>> : end_(other.end_) {}

    constexpr std::ranges::sentinel_t<Base> base() const { return end_; }

    template <bool OtherConst>
    requires std::sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr bool
    operator==(const iterator<OtherConst>& x, const sentinel& y) {
      return y.equal(x);
    }

    template <bool OtherConst>
    requires std::sized_sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr std::ranges::
      range_difference_t<maybe_const<OtherConst, V>>
      operator-(const iterator<OtherConst>& x, const sentinel& y) {
      return -y.distance_from(x);
    }

    template <bool OtherConst>
    requires std::sized_sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr std::ranges::
      range_difference_t<maybe_const<OtherConst, V>>
      operator-(const sentinel& y, const iterator<OtherConst>& x) {
      return y.distance_from(x);
    }
  };

  V base_ = V();

 public:
  elements_view() = default;
  constexpr explicit elements_view(V base) : base_(std::move(base)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() requires(!simple_view<V>) {
    return iterator<false>(std::ranges::begin(base_));
  }

  constexpr auto begin() const requires std::ranges::range<const V> {
    return iterator<true>(std::ranges::begin(base_));
  }

  constexpr auto end() requires(!simple_view<V> && !std::ranges::common_range<V>) {
    return sentinel<false>{std::ranges::end(base_)};
  }

  constexpr auto end() requires(!simple_view<V> && std::ranges::common_range<V>) {
    return iterator<false>{std::ranges::end(base_)};
  }

  constexpr auto end() const requires std::ranges::range<const V> {
    return sentinel<true>{std::ranges::end(base_)};
  }

  constexpr auto end() const requires std::ranges::common_range<const V> {
    return iterator<true>{std::ranges::end(base_)};
  }

  constexpr auto size() requires std::ranges::sized_range<V> { return std::ranges::size(base_); }

  constexpr auto size() const requires std::ranges::sized_range<const V> {
    return std::ranges::size(base_);
  }
};

template <class R>
using keys_view = elements_view<views::all_t<R>, 0>;

template <class R>
using values_view = elements_view<views::all_t<R>, 1>;

}  // namespace ranges

namespace views {

template <std::size_t N, class R>
concept can_elements_view = requires {
  ranges::elements_view<views::all_t<R>, N>{std::declval<R>()};
};

template <std::size_t N>
struct Elements : ranges::RangeAdaptorClosure {
  template <std::ranges::viewable_range R>
  requires can_elements_view<N, R> constexpr auto operator()(R&& r) const {
    return ranges::elements_view<views::all_t<R>, N>{std::forward<R>(r)};
  }
};

template <std::size_t N>
inline constexpr Elements<N> elements;
inline constexpr auto keys = elements<0>;
inline constexpr auto values = elements<1>;

}  // namespace views

template <class T, std::size_t N>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::elements_view<T, N>> =
  std::ranges::enable_borrowed_range<T>;