#pragma once

#include <algorithm>

#include "ref_view.hpp"
#include "single_view.hpp"
#include "subrange.hpp"

namespace ranges {

template <auto>
struct require_constant;

template <class R>
concept tiny_range = std::ranges::sized_range<R>&& requires {
  typename require_constant<std::remove_reference_t<R>::size()>;
}
&&(std::remove_reference_t<R>::size() <= 1);

template <std::ranges::input_range V, std::ranges::forward_range Pattern>
  requires std::ranges::view<V>&& std::ranges::view<Pattern>&& std::indirectly_comparable<
    std::ranges::iterator_t<V>, std::ranges::iterator_t<Pattern>, std::ranges::equal_to> &&
  (std::ranges::forward_range<V> || tiny_range<Pattern>)class split_view
  : public view_interface<split_view<V, Pattern>> {
 private:
  V base_ = V();
  Pattern pattern_ = Pattern();
  [[no_unique_address]] maybe_present_t<!std::ranges::forward_range<V>, std::ranges::iterator_t<V>>
    current_{};

  template <class Base>
  struct outer_iter_cat {};
  template <std::ranges::forward_range Base>
  struct outer_iter_cat<Base> {
    using iterator_category = std::input_iterator_tag;
  };

  template <bool Const>
  struct inner_iterator;

  template <bool Const>
  struct outer_iterator : outer_iter_cat<maybe_const<Const, V>> {
   private:
    using Parent = maybe_const<Const, split_view>;
    using Base = maybe_const<Const, V>;
    Parent* parent_ = nullptr;
    maybe_present_t<std::ranges::forward_range<V>, std::ranges::iterator_t<Base>> current_{};

    friend outer_iterator<!Const>;
    friend inner_iterator<Const>;

    constexpr auto& current() noexcept {
      if constexpr (std::ranges::forward_range<V>)
        return current_;
      else
        return parent_->current_;
    }

    constexpr auto& current() const noexcept {
      if constexpr (std::ranges::forward_range<V>)
        return current_;
      else
        return parent_->current_;
    }

    constexpr bool at_end() const { return current() == std::ranges::end(parent_->base_); }

   public:
    using iterator_concept = std::conditional_t<
      std::ranges::forward_range<Base>, std::forward_iterator_tag, std::input_iterator_tag>;
    using difference_type = std::ranges::range_difference_t<Base>;

    struct value_type : view_interface<value_type> {
     private:
      outer_iterator i_ = outer_iterator();

     public:
      value_type() = default;
      constexpr explicit value_type(outer_iterator i) : i_(std::move(i)) {}

      constexpr inner_iterator<Const> begin() const requires std::copyable<outer_iterator> {
        return inner_iterator<Const>{i_};
      }

      constexpr inner_iterator<Const> begin() requires(!std::copyable<outer_iterator>) {
        return inner_iterator<Const>{std::move(i_)};
      }

      constexpr std::default_sentinel_t end() const { return std::default_sentinel; }
    };

    outer_iterator() = default;

    constexpr explicit outer_iterator(Parent& parent) requires(!std::ranges::forward_range<Base>)
      : parent_(std::addressof(parent)) {}

    constexpr outer_iterator(
      Parent& parent,
      std::ranges::iterator_t<Base> current) requires std::ranges::forward_range<Base>
      : parent_(std::addressof(parent)), current_(std::move(current)) {}

    constexpr outer_iterator(outer_iterator<!Const> i) requires Const&& std::convertible_to<
      std::ranges::iterator_t<V>, std::ranges::iterator_t<Base>>
      : parent_(i.parent_), current_(std::move(i.current_)) {}

    constexpr value_type operator*() const { return value_type{*this}; }

    constexpr outer_iterator& operator++() {
      const auto end = std::ranges::end(parent_->base_);
      if (current() == end) return *this;
      const auto [pbegin, pend] = subrange{parent_->pattern_};
      if (pbegin == pend)
        ++current();
      else if constexpr (tiny_range<Pattern>) {
        current() = std::ranges::find(std::move(current()), end, *pbegin);
        if (current() != end) ++current();
      } else {
        do {
          auto [b, p] = std::ranges::mismatch(current(), end, pbegin, pend);
          if (p == pend) {
            current() = b;
            break;
          }
        } while (++current() != end);
      }
      return *this;
    }

    constexpr decltype(auto) operator++(int) {
      if constexpr (std::ranges::forward_range<Base>) {
        auto tmp = *this;
        ++*this;
        return tmp;
      } else
        ++*this;
    }

    friend constexpr bool operator==(
      const outer_iterator& x, const outer_iterator& y) requires std::ranges::forward_range<Base> {
      return x.current_ == y.current_;
    }

    friend constexpr bool operator==(const outer_iterator& x, std::default_sentinel_t) {
      return x.at_end();
    }
  };

  template <class Base>
  struct inner_iter_cat {};
  template <std::ranges::forward_range Base>
  struct inner_iter_cat<Base> {
   private:
    using C = std::iterator_traits<std::ranges::iterator_t<Base>>::iterator_category;

   public:
    using iterator_category = std::conditional_t<
      std::derived_from<C, std::forward_iterator_tag>, std::forward_iterator_tag, C>;
  };

  template <bool Const>
  struct inner_iterator : inner_iter_cat<maybe_const<Const, V>> {
   private:
    using Base = maybe_const<Const, V>;
    outer_iterator<Const> i_ = outer_iterator<Const>();
    bool incremented_ = false;

    constexpr auto& i_current() noexcept { return i_.current(); }
    constexpr auto& i_current() const noexcept { return i_.current(); }

    constexpr bool at_end() const {
      auto [pcur, pend] = subrange{i_.parent_->pattern_};
      auto end = std::ranges::end(i_.parent_->base_);
      if constexpr (tiny_range<Pattern>) {
        const auto& cur = i_current();
        if (cur == end) return true;
        if (pcur == pend) return incremented_;
        return *cur == *pcur;
      } else {
        auto cur = i_current();
        if (cur == end) return true;
        if (pcur == pend) return incremented_;
        do {
          if (*cur != *pcur) return false;
          if (++pcur == pend) return true;
        } while (++cur != end);
        return false;
      }
    }

   public:
    using iterator_concept = typename outer_iterator<Const>::iterator_concept;
    using value_type = std::ranges::range_value_t<Base>;
    using difference_type = std::ranges::range_difference_t<Base>;

    inner_iterator() = default;
    constexpr explicit inner_iterator(outer_iterator<Const> i) : i_(std::move(i)) {}

    constexpr decltype(auto) operator*() const { return *i_current(); }

    constexpr inner_iterator& operator++() {
      incremented_ = true;
      if constexpr (!std::ranges::forward_range<Base>) {
        if constexpr (Pattern::size() == 0) return *this;
      }
      ++i_current();
      return *this;
    }

    constexpr decltype(auto) operator++(int) {
      if constexpr (std::ranges::forward_range<Base>) {
        auto tmp = *this;
        ++*this;
        return tmp;
      } else
        ++*this;
    }

    friend constexpr bool operator==(
      const inner_iterator& x, const inner_iterator& y) requires std::ranges::forward_range<Base> {
      return x.i_ == y.i_;
    }

    friend constexpr bool operator==(const inner_iterator& x, std::default_sentinel_t) {
      return x.at_end();
    }

    friend constexpr decltype(auto) iter_move(const inner_iterator& i) noexcept(
      noexcept(std::ranges::iter_move(i.i_current()))) {
      return std::ranges::iter_move(i.i_current());
    }

    friend constexpr void iter_swap(const inner_iterator& x, const inner_iterator& y) noexcept(
      noexcept(std::ranges::iter_swap(
        x.i_current(),
        y.i_current()))) requires std::indirectly_swappable<std::ranges::iterator_t<Base>> {
      std::ranges::iter_swap(x.i_current(), y.i_current());
    }
  };

 public:
  split_view() = default;

  constexpr split_view(V base, Pattern pattern)
    : base_(std::move(base)), pattern_(std::move(pattern)) {}

  template <std::ranges::input_range R>
  requires std::constructible_from<V, views::all_t<R>>&& std::constructible_from<
    Pattern, ranges::single_view<std::ranges::range_value_t<
               R>>> constexpr split_view(R&& r, std::ranges::range_value_t<R> e)
    : base_(views::all(std::forward<R>(r))), pattern_(single_view{std::move(e)}) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() {
    if constexpr (std::ranges::forward_range<V>)
      return outer_iterator<simple_view<V>>{*this, std::ranges::begin(base_)};
    else {
      current_ = std::ranges::begin(base_);
      return outer_iterator<false>{*this};
    }
  }

  constexpr auto begin() const
    requires std::ranges::forward_range<V>&& std::ranges::forward_range<const V> {
    return outer_iterator<true>{*this, std::ranges::begin(base_)};
  }

  constexpr auto end() requires std::ranges::forward_range<V>&& std::ranges::common_range<V> {
    return outer_iterator<simple_view<V>>{*this, std::ranges::end(base_)};
  }

  constexpr auto end() const {
    if constexpr (
      std::ranges::forward_range<V> && std::ranges::forward_range<const V> &&
      std::ranges::common_range<const V>)
      return outer_iterator<true>{*this, std::ranges::end(base_)};
    else
      return std::default_sentinel;
  }
};

template <class R, class P>
split_view(R&&, P&&) -> split_view<views::all_t<R>, views::all_t<P>>;

template <std::ranges::input_range R>
split_view(R&&, std::ranges::range_value_t<R>)
  -> split_view<views::all_t<R>, single_view<std::ranges::range_value_t<R>>>;

}  // namespace ranges

namespace views {

template <class R, class Pattern>
concept can_split_view = requires {
  ranges::split_view(std::declval<R>(), std::declval<Pattern>());
};

struct Split : ranges::RangeAdaptor<Split> {
  template <std::ranges::viewable_range R, class Pattern>
  requires can_split_view<R, Pattern> 
  constexpr auto operator()(R&& r, Pattern&& pattern) const {
    return ranges::split_view(std::forward<R>(r), std::forward<Pattern>(pattern));
  }

  using RangeAdaptor<Split>::operator();
  static constexpr int arity = 2;
};

inline constexpr Split split;

}  // namespace views