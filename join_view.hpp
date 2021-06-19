#pragma once

#include "ref_view.hpp"

namespace ranges {

template <std::ranges::input_range V>
  requires std::ranges::view<V>&& std::ranges::input_range<std::ranges::range_reference_t<V>> &&
  (std::is_reference_v<std::ranges::range_reference_t<V>> ||
   std::ranges::view<std::ranges::range_value_t<V>>)class join_view
  : public view_interface<join_view<V>> {
  using InnerRng = std::ranges::range_reference_t<V>;

  template <bool Const>
  using Base = maybe_const<Const, V>;

  template <bool Const>
  static constexpr bool ref_is_glvalue =
    std::is_reference_v<std::ranges::range_reference_t<Base<Const>>>;

  template <bool Const>
  using OuterIter = std::ranges::iterator_t<Base<Const>>;

  template <bool Const>
  using InnerIter = std::ranges::iterator_t<std::ranges::range_reference_t<Base<Const>>>;

  template <bool Const>
  struct iter_cat {};

  template <bool Const>
  requires ref_is_glvalue<Const>&& std::ranges::forward_range<Base<Const>>&&
    std::ranges::forward_range<std::ranges::range_reference_t<Base<Const>>> 
  struct iter_cat<Const> {
   private:
    static constexpr auto iter_category() {
      using OUTERC = typename std::iterator_traits<OuterIter<Const>>::iterator_category;
      using INNERC = typename std::iterator_traits<InnerIter<Const>>::iterator_category;
      if constexpr (
        std::derived_from<OUTERC, std::bidirectional_iterator_tag> &&
        std::derived_from<INNERC, std::bidirectional_iterator_tag>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (
        std::derived_from<OUTERC, std::forward_iterator_tag> &&
        std::derived_from<INNERC, std::forward_iterator_tag>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

   public:
    using iterator_category = decltype(iter_category());
  };

  template <bool Const>
  struct sentinel;

  template <bool Const>
  struct iterator : iter_cat<Const> {
   private:
    using Parent = maybe_const<Const, join_view>;
    using Base = join_view::Base<Const>;

    static constexpr bool ref_is_glvalue = join_view::ref_is_glvalue<Const>;

    // join_­view iterators use the satisfy function to skip over empty inner ranges.
    constexpr void satisfy() {
      auto update_inner = [this](std::ranges::range_reference_t<Base> x) -> auto& {
        if constexpr (ref_is_glvalue)  // x is a reference
          return x;
        else
          return (parent_->inner_ = views::all(std::move(x)));
      };

      for (; outer_ != std::ranges::end(parent_->base_); ++outer_) {
        auto& inner = update_inner(*outer_);
        inner_ = std::ranges::begin(inner);
        if (inner_ != std::ranges::end(inner)) return;
      }

      if constexpr (ref_is_glvalue) inner_ = InnerIter();
    }

    static constexpr auto iter_concept() {
      if constexpr (
        ref_is_glvalue && std::ranges::bidirectional_range<Base> &&
        std::ranges::bidirectional_range<std::ranges::range_reference_t<Base>>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (
        ref_is_glvalue && std::ranges::forward_range<Base> &&
        std::ranges::forward_range<std::ranges::range_reference_t<Base>>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

    using OuterIter = join_view::OuterIter<Const>;
    using InnerIter = join_view::InnerIter<Const>;

    OuterIter outer_ = OuterIter();
    InnerIter inner_ = InnerIter();
    Parent* parent_ = nullptr;

   public:
    using iterator_concept = decltype(iter_concept());
    using value_type = std::ranges::range_value_t<std::ranges::range_reference_t<Base>>;
    using difference_type = std::common_type_t<
      std::ranges::range_difference_t<Base>,
      std::ranges::range_difference_t<std::ranges::range_reference_t<Base>>>;

    iterator() = default;
    constexpr iterator(Parent& parent, OuterIter outer)
      : parent_(std::addressof(parent)), outer_(std::move(outer)) {
      satisfy();
    }
    constexpr iterator(iterator<!Const> i) requires Const&& std::convertible_to<
      std::ranges::iterator_t<V>,
      OuterIter>&& std::convertible_to<std::ranges::iterator_t<InnerRng>, InnerIter>
      : outer_(std::move(i.outer_)), inner_(std::move(i.inner_)), parent_(i.parent_) {}

    constexpr decltype(auto) operator*() const { return *inner_; }

    constexpr InnerIter operator->() const
      requires has_arrow<InnerIter>&& std::copyable<InnerIter> {
      return inner_;
    }

    constexpr iterator& operator++() {
      auto inner_range = [this]() -> auto&& {
        if constexpr (ref_is_glvalue)
          return *outer_;
        else
          return parent_->inner_;
      };

      auto&& inner_rng = inner_range();
      if (++inner_ == std::ranges::end(inner_rng)) {
        ++outer_;
        satisfy();
      }
      return *this;
    }
    constexpr void operator++(int) { ++*this; }
    constexpr iterator operator++(int) requires ref_is_glvalue&& std::ranges::forward_range<
      Base>&& std::ranges::forward_range<std::ranges::range_reference_t<Base>> {
      auto tmp = *this;
      ++*this;
      return tmp;
    }

    constexpr iterator& operator--() requires ref_is_glvalue&& std::ranges::bidirectional_range<
      Base>&& std::ranges::bidirectional_range<std::ranges::range_reference_t<Base>>&& std::ranges::
      common_range<std::ranges::range_reference_t<Base>> {
      if (outer_ == std::ranges::end(parent_->base_)) inner_ = std::ranges::end(*--outer_);
      while (inner_ == std::ranges::begin(*outer_)) inner_ = std::ranges::end(*--outer_);
      --inner_;
      return *this;
    }

    constexpr iterator& operator--(int) requires ref_is_glvalue&& std::ranges::bidirectional_range<
      Base>&& std::ranges::bidirectional_range<std::ranges::range_reference_t<Base>>&& std::ranges::
      common_range<std::ranges::range_reference_t<Base>> {
      auto tmp = *this;
      --*this;
      return tmp;
    }

    friend constexpr bool operator==(
      const iterator& x, const iterator& y) requires ref_is_glvalue&& std::
      equality_comparable<OuterIter>&& std::equality_comparable<InnerIter> {
      return x.outer_ == y.outer_ && x.inner_ == y.inner_;
    }

    friend constexpr decltype(auto) iter_move(const iterator& i) noexcept(
      noexcept(std::ranges::iter_move(i.inner_))) {
      return std::ranges::iter_move(i.inner_);
    }

    friend constexpr void iter_swap(const iterator& x, const iterator& y) noexcept(
      noexcept(std::ranges::iter_swap(x.inner_, y.inner_))) {
      std::ranges::iter_swap(x.inner_, y.inner_);
    }

    friend iterator<!Const>;
    template <bool>
    friend struct sentinel;
  };

  template <bool Const>
  struct sentinel {
   private:
    using Parent = maybe_const<Const, join_view>;
    using Base = join_view::Base<Const>;

    std::ranges::sentinel_t<Base> end_ = std::ranges::sentinel_t<Base>();

    template <bool OtherConst>
    constexpr bool equal(const iterator<OtherConst>& i) const {
      return i.outer_ == end_;
    }

   public:
    sentinel() = default;
    constexpr explicit sentinel(Parent& parent) : end_(std::ranges::end(parent.base_)) {}
    constexpr sentinel(sentinel<!Const> s) requires Const&& std::convertible_to<
      std::ranges::sentinel_t<V>, std::ranges::sentinel_t<Base>> : end_(std::move(s.end_)) {}

    template <bool OtherConst>
    requires std::sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr bool
    operator==(const iterator<OtherConst>& x, const sentinel& y) {
      return y.equal(x);
    }

    friend sentinel<!Const>;
  };

  V base_ = V();
  [[no_unique_address]] maybe_present_t<!std::is_reference_v<InnerRng>, views::all_t<InnerRng>>
    inner_{};

 public:
  join_view() = default;
  constexpr explicit join_view(V base) : base_(std::move(base)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() {
    constexpr bool use_const = simple_view<V> && std::is_reference_v<InnerRng>;
    return iterator<use_const>{*this, std::ranges::begin(base_)};
  }

  constexpr auto begin() const requires std::ranges::input_range<const V>&& std::is_reference_v<
    std::ranges::range_reference_t<const V>> {
    return iterator<true>{*this, std::ranges::begin(base_)};
  }

  constexpr auto end() {
    if constexpr (
      std::ranges::forward_range<V> && std::is_reference_v<InnerRng> &&
      std::ranges::forward_range<InnerRng> && std::ranges::common_range<V> &&
      std::ranges::common_range<InnerRng>)
      return iterator<simple_view<V>>{*this, std::ranges::end(base_)};
    else
      return sentinel<simple_view<V>>{*this};
  }

  constexpr auto end() const requires std::ranges::input_range<const V>&& std::is_reference_v<
    std::ranges::range_reference_t<const V>> {
    if constexpr (
      std::ranges::forward_range<const V> &&
      std::is_reference_v<std::ranges::range_reference_t<const V>> &&
      std::ranges::forward_range<std::ranges::range_reference_t<const V>> &&
      std::ranges::common_range<const V> &&
      std::ranges::common_range<std::ranges::range_reference_t<const V>>)
      return iterator<true>{*this, std::ranges::end(base_)};
    else
      return sentinel<true>{*this};
  }
};

template <class R>
explicit join_view(R&&) -> join_view<views::all_t<R>>;

}  // namespace ranges

namespace views {

template <class R>
concept can_join_view = requires {
  ranges::join_view<views::all_t<R>>{std::declval<R>()};
};

struct Join : ranges::RangeAdaptorClosure {
  template <std::ranges::viewable_range R>
  requires can_join_view<R> constexpr auto operator()(R&& r) const {
    return ranges::join_view<views::all_t<R>>{std::forward<R>(r)};
  }
};

inline constexpr Join join;

}  // namespace views