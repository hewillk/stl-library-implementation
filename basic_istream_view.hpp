#pragma once

#include <istream>

#include "view_interface.hpp"

namespace ranges {

template <class Val, class CharT, class Traits>
concept stream_extractable = requires(std::basic_istream<CharT, Traits>& is, Val& t) {
  is >> t;
};

template <std::movable Val, class CharT, class Traits>
requires std::default_initializable<Val>&&
  stream_extractable<Val, CharT, Traits> class basic_istream_view
  : public view_interface<basic_istream_view<Val, CharT, Traits>> {
 private:
  std::basic_istream<CharT, Traits>* stream_ = nullptr;
  Val value_ = Val();

  struct iterator {
    using iterator_concept = std::input_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = Val;

    iterator() = default;
    constexpr explicit iterator(basic_istream_view& parent) noexcept
      : parent_(std::addressof(parent)) {}

    iterator(const iterator&) = delete;
    iterator(iterator&&) = default;

    iterator& operator=(const iterator&) = delete;
    iterator& operator=(iterator&&) = default;

    iterator& operator++() {
      assert(parent_->stream_ != nullptr);
      *parent_->stream_ >> parent_->value_;
      return *this;
    }
    void operator++(int) { ++*this; }

    Val& operator*() const {
      assert(parent_->stream_ != nullptr);
      return parent_->value_;
    }

    friend bool operator==(const iterator& x, std::default_sentinel_t) { return x.at_end(); }

   private:
    basic_istream_view* parent_ = nullptr;

    bool at_end() const { return parent_ == nullptr || !*parent_->stream_; }
  };

 public:
  basic_istream_view() = default;
  constexpr explicit basic_istream_view(std::basic_istream<CharT, Traits>& stream)
    : stream_(std::addressof(stream)) {}

  constexpr auto begin() {
    if (stream_) *stream_ >> value_;
    return iterator{*this};
  }

  constexpr std::default_sentinel_t end() const noexcept { return std::default_sentinel; }
};

template <class Val, class CharT, class Traits>
basic_istream_view<Val, CharT, Traits> istream_view(std::basic_istream<CharT, Traits>& s) {
  return basic_istream_view<Val, CharT, Traits>{s};
}

}  // namespace ranges