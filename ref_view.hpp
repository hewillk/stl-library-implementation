#pragma once

#include "adaptor.hpp"
#include "subrange.hpp"

namespace ranges {

template <std::ranges::range R>
requires std::is_object_v<R> class ref_view : public view_interface<ref_view<R>> {
 private:
  R* r_ = nullptr;

  void FUN(R&);
  void FUN(R&&) = delete;

 public:
  constexpr ref_view() noexcept = default;

  template <not_same_as<ref_view> T>
  requires std::convertible_to<T, R&>&& requires {
    FUN(std::declval<T>());
  }
  constexpr ref_view(T&& t) : r_(std::addressof(static_cast<R&>(std::forward<T>(t)))) {}

  constexpr R& base() const { return *r_; }

  constexpr std::ranges::iterator_t<R> begin() const { return std::ranges::begin(*r_); }

  constexpr std::ranges::sentinel_t<R> end() const { return std::ranges::end(*r_); }

  constexpr bool empty() const requires requires { std::ranges::empty(*r_); }
  { return std::ranges::empty(*r_); }

  constexpr auto size() const requires std::ranges::sized_range<R> {
    return std::ranges::size(*r_);
  }

  constexpr auto data() const requires std::ranges::contiguous_range<R> {
    return std::ranges::data(*r_);
  }
};

template <class R>
ref_view(R&) -> ref_view<R>;

}  // namespace ranges

namespace views {

template <class R>
concept can_ref_view = requires {
  ranges::ref_view{std::declval<R>()};
};

struct All : ranges::RangeAdaptorClosure {
  template <std::ranges::viewable_range R>
  constexpr auto operator()(R&& r) const {
    if constexpr (std::ranges::view<std::decay_t<R>>)
      return std::forward<R>(r);
    else if constexpr (can_ref_view<R>)
      return ranges::ref_view{std::forward<R>(r)};
    else
      return ranges::subrange{std::forward<R>(r)};
  }
};

inline constexpr All all{};

template <std::ranges::viewable_range R>
using all_t = decltype(all(std::declval<R>()));

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::ref_view<T>> = true;