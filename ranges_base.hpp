#pragma once

#include <cassert>
#include <concepts>
#include <ranges>

namespace ranges::detail {

template <class T>
concept class_or_enum = std::is_class_v<T> || std::is_enum_v<T> || std::is_union_v<T>;

template <class T>
concept is_integer_like = std::integral<T> && !std::same_as<std::remove_cv_t<T>, bool>;

template <std::integral T>
constexpr auto to_unsigned_like(T t) noexcept {
  return static_cast<std::make_unsigned_t<T>>(t);
}

template <class T>
constexpr std::decay_t<T> decay_copy(T&& t) noexcept(
  std::is_nothrow_convertible_v<T, std::decay_t<T>>) {
  return std::forward<T>(t);
}

template <class T>
concept maybe_borrowed_range =
  std::is_lvalue_reference_v<T> || std::ranges::enable_borrowed_range<std::remove_cvref_t<T>>;

}  // namespace ranges::detail

namespace ranges::access {

template <class T>
concept member_begin = requires(T& t) {
  { detail::decay_copy(t.begin()) }
  ->std::input_or_output_iterator;
};

void begin(auto&) = delete;
void begin(const auto&) = delete;

template <class T>
concept adl_begin = detail::class_or_enum<std::remove_reference_t<T>>&& requires(T& t) {
  { detail::decay_copy(begin(t)) }
  ->std::input_or_output_iterator;
};

struct Begin {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (std::is_array_v<std::remove_reference_t<T>>)
      return true;
    else if constexpr (member_begin<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().begin()));
    else
      return noexcept(detail::decay_copy(begin(std::declval<T&>())));
  }

 public:
  template <detail::maybe_borrowed_range T>
    requires std::is_array_v<std::remove_reference_t<T>> || member_begin<T> ||
    adl_begin<T> constexpr auto operator()(T&& t) const noexcept(_noexcept<T>()) {
    if constexpr (std::is_array_v<std::remove_reference_t<T>>)
      return t + 0;
    else if constexpr (member_begin<T>)
      return t.begin();
    else
      return begin(t);
  }
};

template <class T>
concept member_end = requires(T& t) {
  { detail::decay_copy(t.end()) }
  ->std::sentinel_for<decltype(Begin{}(t))>;
};

void end(auto&) = delete;
void end(const auto&) = delete;

template <class T>
concept adl_end = detail::class_or_enum<std::remove_reference_t<T>>&& requires(T& t) {
  { detail::decay_copy(end(t)) }
  ->std::sentinel_for<decltype(Begin{}(t))>;
};

struct End {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (std::is_bounded_array_v<std::remove_reference_t<T>>)
      return true;
    else if constexpr (member_end<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().end()));
    else
      return noexcept(detail::decay_copy(end(std::declval<T&>())));
  }

 public:
  template <detail::maybe_borrowed_range T>
    requires std::is_bounded_array_v<std::remove_reference_t<T>> || member_end<T> ||
    adl_end<T> constexpr auto operator()(T&& t) const noexcept(_noexcept<T>()) {
    if constexpr (std::is_bounded_array_v<std::remove_reference_t<T>>)
      return t + std::extent_v<std::remove_reference_t<T>>;
    else if constexpr (member_end<T>)
      return t.end();
    else
      return end(t);
  }
};

template <class T>
constexpr decltype(auto) as_const(T&& t) noexcept {
  if constexpr (std::is_lvalue_reference_v<T>)
    return static_cast<const T&>(t);
  else
    return static_cast<const T&&>(t);
}

struct CBegin {
  template <class T>
  constexpr auto operator()(T&& t) const
    noexcept(noexcept(Begin{}(as_const(std::forward<T>(t))))) requires requires {
    Begin{}(as_const(std::forward<T>(t)));
  }
  { return Begin{}(as_const(std::forward<T>(t))); }
};

struct CEnd {
  template <class T>
  constexpr auto operator()(T&& t) const
    noexcept(noexcept(End{}(as_const(std::forward<T>(t))))) requires requires {
    End{}(as_const(std::forward<T>(t)));
  }
  { return End{}(as_const(std::forward<T>(t))); }
};

template <class T>
concept member_rbegin = requires(T& t) {
  { detail::decay_copy(t.rbegin()) }
  ->std::input_or_output_iterator;
};

void rbegin(auto&) = delete;
void rbegin(const auto&) = delete;

template <class T>
concept adl_rbegin = detail::class_or_enum<std::remove_reference_t<T>>&& requires(T& t) {
  { detail::decay_copy(rbegin(t)) }
  ->std::input_or_output_iterator;
};

template <class T>
concept reversable = requires(T& t) {
  { Begin{}(t) }
  ->std::bidirectional_iterator;
  { End{}(t) }
  ->std::same_as<decltype(Begin{}(t))>;
};

struct RBegin {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (member_rbegin<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().rbegin()));
    else if constexpr (adl_rbegin<T>)
      return noexcept(detail::decay_copy(rbegin(std::declval<T&>())));
    else {
      if constexpr (noexcept(End{}(std::declval<T&>())))
        return std::is_nothrow_copy_constructible_v<decltype(End{}(std::declval<T&>()))>;
      else
        return false;
    }
  }

 public:
  template <detail::maybe_borrowed_range T>
    requires member_rbegin<T> || adl_rbegin<T> ||
    reversable<T> constexpr auto operator()(T&& t) const noexcept(_noexcept<T>()) {
    if constexpr (member_rbegin<T>)
      return t.rbegin();
    else if constexpr (adl_rbegin<T>)
      return rbegin(t);
    else
      return std::make_reverse_iterator(End{}(t));
  }
};

template <class T>
concept member_rend = requires(T& t) {
  { detail::decay_copy(t.rend()) }
  ->std::sentinel_for<decltype(RBegin{}(t))>;
};

void rend(auto&) = delete;
void rend(const auto&) = delete;

template <class T>
concept adl_rend = detail::class_or_enum<std::remove_reference_t<T>>&& requires(T& t) {
  { detail::decay_copy(rend(t)) }
  ->std::sentinel_for<decltype(RBegin{}(t))>;
};

struct REnd {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (member_rend<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().rend()));
    else if constexpr (adl_rend<T>)
      return noexcept(detail::decay_copy(rend(std::declval<T&>())));
    else {
      if constexpr (noexcept(Begin{}(std::declval<T&>())))
        return std::is_nothrow_copy_constructible_v<decltype(Begin{}(std::declval<T&>()))>;
      else
        return false;
    }
  }

 public:
  template <detail::maybe_borrowed_range T>
    requires member_rend<T> || adl_rend<T> ||
    reversable<T> constexpr auto operator()(T&& t) const noexcept(_noexcept<T>()) {
    if constexpr (member_rend<T>)
      return t.rend();
    else if constexpr (adl_rend<T>)
      return rend(t);
    else
      return std::make_reverse_iterator(Begin{}(t));
  }
};

struct CRBegin {
  template <class T>
  constexpr auto operator()(T&& t) const
    noexcept(noexcept(RBegin{}(as_const(std::forward<T>(t))))) requires requires {
    RBegin{}(as_const(std::forward<T>(t)));
  }
  { return RBegin{}(as_const(std::forward<T>(t))); }
};

struct CREnd {
  template <class T>
  constexpr auto operator()(T&& t) const
    noexcept(noexcept(REnd{}(as_const(std::forward<T>(t))))) requires requires {
    REnd{}(as_const(std::forward<T>(t)));
  }
  { return REnd{}(as_const(std::forward<T>(t))); }
};

template <class T>
concept member_size = !std::ranges::disable_sized_range<std::remove_cvref_t<T>> && requires(T & t) {
  { detail::decay_copy(t.size()) }
  ->detail::is_integer_like;
};

void size(auto&) = delete;
void size(const auto&) = delete;

template <class T>
concept adl_size = detail::class_or_enum<std::remove_reference_t<T>> &&
                   !std::ranges::disable_sized_range<std::remove_cvref_t<T>> && requires(T & t) {
  { detail::decay_copy(size(t)) }
  ->detail::is_integer_like;
};

template <class T>
concept sentinel_size = requires(T& t) {
  { Begin{}(t) }
  ->std::forward_iterator;
  { End{}(t) }
  ->std::sized_sentinel_for<decltype(Begin{}(t))>;
  to_unsigned_like(End{}(t)-Begin{}(t));
};

struct Size {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (std::is_bounded_array_v<std::remove_reference_t<T>>)
      return true;
    else if constexpr (member_size<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().size()));
    else if constexpr (adl_size<T>)
      return noexcept(detail::decay_copy(size(std::declval<T&>())));
    else
      return noexcept(End{}(std::declval<T&>()) - Begin{}(std::declval<T&>()));
  }

 public:
  template <class T>
    requires std::is_bounded_array_v<std::remove_reference_t<T>> || member_size<T> || adl_size<T> ||
    sentinel_size<T> constexpr auto operator()(T&& t) const noexcept(_noexcept<T>()) {
    if constexpr (std::is_bounded_array_v<std::remove_reference_t<T>>)
      return std::extent_v<std::remove_reference_t<T>>;
    else if constexpr (member_size<T>)
      return t.size();
    else if constexpr (adl_size<T>)
      return size(t);
    else
      return to_unsigned_like(End{}(t)-Begin{}(t));
  }
};

struct SSize {
  template <class T>
  constexpr auto operator()(T&& t) const noexcept(noexcept(Size{}(t))) requires requires {
    Size{}(t);
  }
  {
    using D = std::make_signed_t<decltype(Size{}(t))>;
    return static_cast<std::conditional_t<sizeof(D) < sizeof(std::ptrdiff_t), std::ptrdiff_t, D>>(
      Size{}(t));
  }
};

template <class T>
concept member_empty = requires(T& t) {
  bool(t.empty());
};

template <class T>
concept size0_empty = requires(T& t) {
  Size{}(t) == 0;
};

template <class T>
concept eq_iter_empty = requires(T& t) {
  { Begin{}(t) }
  ->std::forward_iterator;
  bool(Begin{}(t) == End{}(t));
};

struct Empty {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (member_empty<T>)
      return noexcept(bool(std::declval<T&>().empty()));
    else if constexpr (size0_empty<T>)
      return noexcept(Size{}(std::declval<T&>()) == 0);
    else
      return noexcept(bool(Begin{}(std::declval<T&>()) == End{}(std::declval<T&>())));
  }

 public:
  template <class T>
    constexpr bool operator()(T&& t) const noexcept(_noexcept<T>) requires member_empty<T> ||
    size0_empty<T> || eq_iter_empty<T> {
    if constexpr (member_empty<T>)
      return bool(t.empty());
    else if constexpr (size0_empty<T>)
      return Size{}(t) == 0;
    else
      return bool(Begin{}(t) == End{}(t));
  }
};

template <class T>
concept pointer_to_object = std::is_pointer_v<T>&& std::is_object_v<std::remove_pointer_t<T>>;

template <class T>
concept member_data = requires(T& t) {
  { detail::decay_copy(t.data()) }
  ->pointer_to_object;
};

template <class T>
concept begin_data = requires(T& t) {
  { Begin{}(t) }
  ->std::contiguous_iterator;
};

struct Data {
 private:
  template <class T>
  static constexpr bool _noexcept() {
    if constexpr (member_data<T>)
      return noexcept(detail::decay_copy(std::declval<T&>().data()));
    else
      return noexcept(Begin{}(std::declval<T&>()));
  }

 public:
  template <detail::maybe_borrowed_range T>
    requires member_data<T> || begin_data<T> constexpr auto operator()(T&& t) const
    noexcept(_noexcept<T>()) {
    if constexpr (member_data<T>)
      return detail::decay_copy(t.data());
    else
      return std::to_address(Begin{}(t));
  }
};

struct CData {
  template <class T>
  constexpr auto operator()(T&& t) const
    noexcept(noexcept(Data{}(as_const(std::forward<T>(t))))) requires requires {
    Data{}(as_const(std::forward<T>(t)));
  }
  { return Data{}(as_const(std::forward<T>(t))); }
};

}  // namespace ranges::access

namespace ranges {
inline constexpr access::Begin begin;
inline constexpr access::End end;
inline constexpr access::CBegin cbegin;
inline constexpr access::CEnd cend;
inline constexpr access::RBegin rbegin;
inline constexpr access::REnd rend;
inline constexpr access::CRBegin crbegin;
inline constexpr access::CREnd crend;
inline constexpr access::Size size;
inline constexpr access::SSize ssize;
inline constexpr access::Empty empty;
inline constexpr access::Data data;
inline constexpr access::CData cdata;
}  // namespace ranges

namespace ranges {

template <class T>
concept range = requires(T& t) {
  ranges::begin(t);
  ranges::end(t);
};

template <class T>
using iterator_t = decltype(ranges::begin(std::declval<T&>()));

template <range R>
using sentinel_t = decltype(ranges::end(std::declval<R&>()));

template <class T>
concept sized_range = range<T>&& requires(T& t) {
  ranges::size(t);
};

template <sized_range R>
using range_size_t = decltype(ranges::size(std::declval<R&>()));

template <class R, class T>
concept output_range = range<R>&& std::output_iterator<iterator_t<R>, T>;

template <class T>
concept input_range = range<T>&& std::input_iterator<iterator_t<T>>;

template <class T>
concept forward_range = input_range<T>&& std::forward_iterator<iterator_t<T>>;

template <class T>
concept bidirectional_range = forward_range<T>&& std::bidirectional_iterator<iterator_t<T>>;

template <class T>
concept random_access_range = bidirectional_range<T>&& std::random_access_iterator<iterator_t<T>>;

template <class T>
concept contiguous_range =
  random_access_range<T>&& std::contiguous_iterator<iterator_t<T>>&& requires(T& t) {
  { ranges::data(t) }
  ->std::same_as<std::add_pointer_t<std::ranges::range_reference_t<T>>>;
};

template <class T>
concept common_range =
  range<T>&& std::same_as<std::ranges::iterator_t<T>, std::ranges::sentinel_t<T>>;

}  // namespace ranges

namespace ranges {

struct Advance {
  template <std::input_or_output_iterator I>
  constexpr void operator()(I& i, std::iter_difference_t<I> n) const {
    if constexpr (std::random_access_iterator<I>)
      i += n;
    else if constexpr (std::bidirectional_iterator<I>) {
      if (n > 0) {
        do ++i;
        while (--n);
      } else if (n < 0) {
        do --i;
        while (++n);
      }
    } else {
      assert(n >= 0);
      while (n-- > 0) ++i;
    }
  }

  template <std::input_or_output_iterator I, std::sentinel_for<I> S>
  constexpr void operator()(I& i, S bound) const {
    if constexpr (std::assignable_from<I&, S>)
      i = std::move(bound);
    else if constexpr (std::sized_sentinel_for<S, I>)
      (*this)(i, bound - i);
    else
      while (i != bound) ++i;
  }

  template <std::input_or_output_iterator I, std::sentinel_for<I> S>
  constexpr std::iter_difference_t<I> operator()(I& i, std::iter_difference_t<I> n, S bound) const {
    if constexpr (std::sized_sentinel_for<S, I>) {
      const auto diff = bound - i;
      const auto absdiff = diff < 0 ? -diff : diff;
      const auto absn = n < 0 ? -n : n;
      if (absn >= absdiff) {
        (*this)(i, bound);
        return n - diff;
      } else {
        (*this)(i, n);
        return 0;
      }
    } else if (i == bound || n == 0)
      return 0;
    else if (n > 0) {
      std::iter_difference_t<I> m = 0;
      do {
        ++i;
        ++m;
      } while (m != n && i != bound);
      return n - m;
    } else if constexpr (std::bidirectional_iterator<I> && std::same_as<I, S>) {
      std::iter_difference_t<I> m = 0;
      do {
        --i;
        --m;
      } while (m != n && i != bound);
      return n - m;
    } else {
      assert(n >= 0);
      return n;
    }
  }
};

inline constexpr Advance advance;

struct Distance {
  template <std::input_or_output_iterator I, std::sentinel_for<I> S>
  constexpr std::iter_difference_t<I> operator()(I first, S last) const {
    if constexpr (std::sized_sentinel_for<S, I>)
      return last - first;
    else {
      std::iter_difference_t<I> n = 0;
      while (first != last) {
        ++first;
        ++n;
      }
      return n;
    }
  }

  template <range R>
  constexpr std::ranges::range_difference_t<R> operator()(R&& r) const {
    if constexpr (sized_range<R>)
      return static_cast<std::ranges::range_difference_t<R>>(ranges::size(r));
    else
      return (*this)(ranges::begin(r), ranges::end(r));
  }
};

inline constexpr Distance distance;

struct Next {
  template <std::input_or_output_iterator I>
  constexpr I operator()(I x) const {
    ++x;
    return x;
  }

  template <std::input_or_output_iterator I>
  constexpr I operator()(I x, std::iter_difference_t<I> n) const {
    ranges::advance(x, n);
    return x;
  }

  template <std::input_or_output_iterator I, std::sentinel_for<I> S>
  constexpr I operator()(I x, S bound) const {
    ranges::advance(x, bound);
    return x;
  }

  template <std::input_or_output_iterator I, std::sentinel_for<I> S>
  constexpr I operator()(I x, std::iter_difference_t<I> n, S bound) const {
    ranges::advance(x, n, bound);
    return x;
  }
};

inline constexpr Next next;

struct Prev {
  template <std::bidirectional_iterator I>
  constexpr I operator()(I x) const {
    --x;
    return x;
  }

  template <std::bidirectional_iterator I>
  constexpr I operator()(I x, std::iter_difference_t<I> n) const {
    ranges::advance(x, -n);
    return x;
  }

  template <std::bidirectional_iterator I>
  constexpr I operator()(I x, std::iter_difference_t<I> n, I bound) const {
    ranges::advance(x, -n, bound);
    return x;
  }
};

inline constexpr Prev prev;

}  // namespace ranges