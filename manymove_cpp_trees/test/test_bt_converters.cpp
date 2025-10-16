// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "manymove_cpp_trees/bt_converters.hpp"

TEST(BTConverters, ParsePoseFromString)
{
  const std::string s =
    "position: {x: 0.15, y: -0.2, z: 0.3}, orientation: {x: 0.1, y: 0.2, z: 0.3, w: 0.9}";
  geometry_msgs::msg::Pose p = BT::convertFromString<geometry_msgs::msg::Pose>(s);
  EXPECT_NEAR(p.position.x, 0.15, 1e-9);
  EXPECT_NEAR(p.position.y, -0.2, 1e-9);
  EXPECT_NEAR(p.position.z, 0.3, 1e-9);
  EXPECT_NEAR(p.orientation.x, 0.1, 1e-9);
  EXPECT_NEAR(p.orientation.y, 0.2, 1e-9);
  EXPECT_NEAR(p.orientation.z, 0.3, 1e-9);
  EXPECT_NEAR(p.orientation.w, 0.9, 1e-9);
}

TEST(BTConverters, ParseVectorDouble)
{
  const std::string s = "[1,2,3.5]";
  auto v = BT::convertFromString<std::vector<double>>(s);
  ASSERT_EQ(v.size(), 3u);
  EXPECT_DOUBLE_EQ(v[0], 1.0);
  EXPECT_DOUBLE_EQ(v[1], 2.0);
  EXPECT_DOUBLE_EQ(v[2], 3.5);
  EXPECT_EQ(BT::convertToString(v), s);
}

TEST(BTConverters, ParseVectorString)
{
  const std::string s = "[foo,bar,baz]";
  auto v = BT::convertFromString<std::vector<std::string>>(s);
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], "foo");
  EXPECT_EQ(v[1], "bar");
  EXPECT_EQ(v[2], "baz");
  EXPECT_EQ(BT::convertToString(v), s);
}

TEST(BTConverters, RejectMalformedVector)
{
  EXPECT_THROW(BT::convertFromString<std::vector<double>>("1,2,3]"), BT::RuntimeError);
  EXPECT_THROW(BT::convertFromString<std::vector<std::string>>("foo,bar]"), BT::RuntimeError);
}
