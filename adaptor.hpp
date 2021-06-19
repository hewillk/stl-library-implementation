#pragma once

#include <cassert>
#include <concepts>
#include <tuple>
#include <utility>

namespace ranges {

template <class Adaptor, class... Args>
concept adaptor_invocable = requires {
  std::declval<Adaptor>()(std::declval<Args>()...);
};

template <class Adaptor, class Args>
struct Partial;

template <class Lhs, class Rhs>
struct Pipe;

struct RangeAdaptorClosure {
  template <class Self, class Range>
  requires std::derived_from<std::remove_cvref_t<Self>, RangeAdaptorClosure>&&
    adaptor_invocable<Self, Range>
  friend constexpr auto
    operator|(Range&& r, Self&& self) {
    return std::forward<Self>(self)(std::forward<Range>(r));
  }

  template <class Lhs, class Rhs>
  requires std::derived_from<Lhs, RangeAdaptorClosure>&&
    std::derived_from<Rhs, RangeAdaptorClosure> friend constexpr auto
    operator|(Lhs lhs, Rhs rhs) {
    return Pipe<Lhs, Rhs>{std::move(lhs), std::move(rhs)};
  }
};

template <class Adaptor, class... Args>
concept adaptor_partially_applicable = (Adaptor::arity > 1) &&
                                       (sizeof...(Args) == Adaptor::arity - 1) &&
                                       (std::constructible_from<std::decay_t<Args>, Args> && ...);

template <class Derived>
struct RangeAdaptor {
  template <class... Args>
  requires adaptor_partially_applicable<Derived, Args...> constexpr auto operator()(
    Args&&... args) const {
    return Partial<Derived, std::decay_t<Args>...>{std::forward<Args>(args)...};
  }
};

template <class Adaptor, class Arg>
struct Partial : RangeAdaptorClosure {
  Arg arg_;

  constexpr Partial(Arg arg) : arg_(std::move(arg)) {}

  template <class Range>
  requires adaptor_invocable<Adaptor, Range, const Arg&>
  constexpr auto operator()(
    Range&& r) const& {
    return Adaptor{}(std::forward<Range>(r), arg_);
  }

  template <class Range>
  requires adaptor_invocable<Adaptor, Range, Arg>
  constexpr auto operator()(Range&& r) && {
    return Adaptor{}(std::forward<Range>(r), std::move(arg_));
  }

   template <class Range>
   constexpr auto operator()(Range&& r) const&& = delete;
};

template <class Lhs, class Rhs, class Range>
concept pipe_invocable = requires {
  std::declval<Rhs>()(std::declval<Lhs>()(std::declval<Range>()));
};

template <class Lhs, class Rhs>
struct Pipe : RangeAdaptorClosure {
  [[no_unique_address]] Lhs lhs_;
  [[no_unique_address]] Rhs rhs_;

  constexpr Pipe(Lhs lhs, Rhs rhs) : lhs_(std::move(lhs)), rhs_(std::move(rhs)) {}

  template <class Range>
  requires pipe_invocable<const Lhs&, const Rhs&, Range> 
  constexpr auto operator()(
    Range&& r) const& {
    return rhs_(lhs_(std::forward<Range>(r)));
  }

  template <class Range>
  requires pipe_invocable<Lhs, Rhs, Range> 
  constexpr auto operator()(Range&& r) && {
    return std::move(rhs_)(std::move(lhs_)(std::forward<Range>(r)));
  }

   template <class Range>
   constexpr auto operator()(Range&& r) const&& = delete;
};

}  // namespace ranges