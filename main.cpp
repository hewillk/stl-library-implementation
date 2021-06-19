#include "all.hpp"
#include <string>

int main() {
  std::string s = "hello", s1 = "l";
  views::split(s, s1);
  s | views::split(s1);
  auto adapt = views::split(s1);
  s | adapt;
}