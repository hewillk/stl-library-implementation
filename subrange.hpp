#pragma once

#include "view_interface.hpp"

namespace ranges {

template <class From, class To>
concept convertible_to_non_slicing =
  std::convertible_to<From, To> &&
  !(std::is_pointer_v<std::decay_t<From>> && std::is_pointer_v<std::decay_t<To>> &&
    not_same_as<
      std::remove_pointer_t<std::decay_t<From>>, std::remove_pointer_t<std::decay_t<To>>>);

template <class T>
concept pair_like = !std::is_reference_v<T> && requires(T t) {
  typename std::tuple_size<T>::type;
  requires std::derived_from<std::tuple_size<T>, std::integral_constant<std::size_t, 2>>;
  typename std::tuple_element_t<0, std::remove_const_t<T>>;
  typename std::tuple_element_t<1, std::remove_const_t<T>>;
  { std::get<0>(t) }
  ->std::convertible_to<const std::tuple_element_t<0, T>&>;
  { std::get<1>(t) }
  ->std::convertible_to<const std::tuple_element_t<1, T>&>;
};

template <class T, class U, class V>
concept pair_like_convertible_from =
  !std::ranges::range<T> && pair_like<T> && std::constructible_from<T, U, V> &&
  convertible_to_non_slicing<U, std::tuple_element_t<0, T>> &&
  std::convertible_to<V, std::tuple_element_t<1, T>>;

enum class subrange_kind : bool { unsized, sized };

template <
  std::input_or_output_iterator I, std::sentinel_for<I> S = I,
  subrange_kind K = std::sized_sentinel_for<S, I> ? subrange_kind::sized : subrange_kind::unsized>
requires(K == subrange_kind::sized || !std::sized_sentinel_for<S, I>) class subrange
    : public view_interface<subrange<I, S, K>> {
 private:
  static constexpr bool StoreSize = K == subrange_kind::sized && !std::sized_sentinel_for<S, I>;
  I begin_ = I();
  S end_ = S();
  make_unsigned_like_t<std::iter_difference_t<I>> size_ = 0;

 public:
  subrange() = default;

  constexpr subrange(convertible_to_non_slicing<I> auto i, S s) requires(!StoreSize)
      : begin_(std::move(i)), end_(s) {}

  constexpr subrange(
    convertible_to_non_slicing<I> auto i, S s,
    make_unsigned_like_t<std::iter_difference_t<I>> n) requires(K == subrange_kind::sized)
      : begin_(std::move(i)), end_(s) {
    assert(n == to_unsigned_like(std::ranges::distance(i, s)));
    if constexpr (StoreSize) size_ = n;
  }

  template <not_same_as<subrange> R>
  requires std::ranges::borrowed_range<R>&&
    convertible_to_non_slicing<std::ranges::iterator_t<R>, I>&&
      std::convertible_to<std::ranges::sentinel_t<R>, S> constexpr subrange(
        R&& r) requires StoreSize&& std::ranges::sized_range<R>
      : subrange{r, std::ranges::size(r)} {}

  template <not_same_as<subrange> R>
  requires std::ranges::borrowed_range<R>&&
    convertible_to_non_slicing<std::ranges::iterator_t<R>, I>&&
      std::convertible_to<std::ranges::sentinel_t<R>, S> constexpr subrange(R&& r) requires(
        !StoreSize)
      : subrange{std::ranges::begin(r), std::ranges::end(r)} {}

  template <std::ranges::borrowed_range R>
  requires convertible_to_non_slicing<std::ranges::iterator_t<R>, I>&&
    std::convertible_to<std::ranges::sentinel_t<R>, S> constexpr subrange(
      R&& r, make_unsigned_like_t<std::iter_difference_t<I>> n) requires(K == subrange_kind::sized)
      : subrange{std::ranges::begin(r), std::ranges::end(r), n} {}

  template <not_same_as<subrange> PairLike>
  requires pair_like_convertible_from<PairLike, const I&, const S&> constexpr operator PairLike()
    const {
    return PairLike(begin_, end_);
  }

  constexpr I begin() const requires std::copyable<I> { return begin_; }
  [[nodiscard]] constexpr I begin() requires(!std::copyable<I>) { return std::move(begin_); }
  constexpr S end() const { return end_; }

  constexpr bool empty() const { return begin_ == end_; }
  constexpr make_unsigned_like_t<std::iter_difference_t<I>> size() const
    requires(K == subrange_kind::sized) {
    if constexpr (StoreSize)
      return size_;
    else
      return to_unsigned_like(end_ - begin_);
  }

  [[nodiscard]] constexpr subrange next(
    std::iter_difference_t<I> n = 1) const& requires std::forward_iterator<I> {
    auto tmp = *this;
    tmp.advance(n);
    return tmp;
  };

  [[nodiscard]] constexpr subrange next(std::iter_difference_t<I> n = 1) && {
    advance(n);
    return std::move(*this);
  };

  [[nodiscard]] constexpr subrange prev(std::iter_difference_t<I> n = 1) const
    requires std::bidirectional_iterator<I> {
    auto tmp = *this;
    tmp.advance(-n);
    return tmp;
  }

  constexpr subrange& advance(std::iter_difference_t<I> n) {
    if constexpr (std::bidirectional_iterator<I>) {
      if (n < 0) {
        std::ranges::advance(begin_, n);
        if constexpr (StoreSize) size_ += to_unsigned_like(-n);
        return *this;
      }
    }

    assert(n >= 0);
    auto d = n - std::ranges::advance(begin_, n, end_);
    if constexpr (StoreSize) size_ -= to_unsigned_like(d);
    return *this;
  }
};

template <std::input_or_output_iterator I, std::sentinel_for<I> S>
subrange(I, S) -> subrange<I, S>;

template <std::input_or_output_iterator I, std::sentinel_for<I> S>
subrange(I, S, make_unsigned_like_t<std::iter_difference_t<I>>)
  -> subrange<I, S, subrange_kind::sized>;

template <std::ranges::borrowed_range R>
subrange(R&&) -> subrange<
  std::ranges::iterator_t<R>, std::ranges::sentinel_t<R>,
  (std::ranges::sized_range<R> ||
   std::sized_sentinel_for<std::ranges::sentinel_t<R>, std::ranges::iterator_t<R>>)
    ? subrange_kind::sized
    : subrange_kind::unsized>;

template <std::ranges::borrowed_range R>
subrange(R&&, make_unsigned_like_t<std::ranges::range_difference_t<R>>)
  -> subrange<std::ranges::iterator_t<R>, std::ranges::sentinel_t<R>, subrange_kind::sized>;

template <std::size_t N, class I, class S, subrange_kind K>
requires(N < 2) constexpr auto get(const subrange<I, S, K>& r) {
  if constexpr (N == 0)
    return r.begin();
  else
    return r.end();
}

template <std::size_t N, class I, class S, subrange_kind K>
requires(N < 2) constexpr auto get(subrange<I, S, K>&& r) {
  if constexpr (N == 0)
    return r.begin();
  else
    return r.end();
}

}  // namespace ranges

namespace std {
using std::ranges::get;
}

template <class I, class S, ranges::subrange_kind K>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::subrange<I, S, K>> = true;

template <class I, class S, ranges::subrange_kind K>
struct std::tuple_size<ranges::subrange<I, S, K>> : std::integral_constant<std::size_t, 2> {};

template <class I, class S, ranges::subrange_kind K>
struct std::tuple_element<0, ranges::subrange<I, S, K>> {
  using type = I;
};

template <class I, class S, ranges::subrange_kind K>
struct std::tuple_element<1, ranges::subrange<I, S, K>> {
  using type = S;
};

template <class I, class S, ranges::subrange_kind K>
struct std::tuple_element<0, const ranges::subrange<I, S, K>> {
  using type = I;
};

template <class I, class S, ranges::subrange_kind K>
struct std::tuple_element<1, const ranges::subrange<I, S, K>> {
  using type = S;
};