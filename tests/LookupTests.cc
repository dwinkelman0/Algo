// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <algo/Lookup.h>
#include <gtest/gtest.h>

namespace algo {
TEST(Lookup, Subscript) {
  Lookup<std::string> lookup;
  ASSERT_EQ(lookup("hello"), 0);
  ASSERT_EQ(lookup("world"), 1);
  ASSERT_EQ(lookup(static_cast<uint32_t>(0)), "hello");
  ASSERT_EQ(lookup(1), "world");
  ASSERT_THROW(lookup(2), std::out_of_range);
}
}  // namespace algo
