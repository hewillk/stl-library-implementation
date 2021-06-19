#pragma once

#include "subrange.hpp"

namespace views {

struct Counted {
  template <std::input_or_output_iterator I>
  constexpr auto operator()(I i, std::iter_difference_t<I> n) const {
    if constexpr (std::random_access_iterator<I>)
      return ranges::subrange(i, i + n);
    else
      return ranges::subrange(std::counted_iterator(std::move(i), n), std::default_sentinel);
  }
};

inline constexpr Counted counted;

}  // namespace views