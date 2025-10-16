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

#include <QApplication>
#include <QCoreApplication>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <vector>

#include "manymove_hmi/hmi_gui.hpp"

namespace
{
void pump()
{
  QCoreApplication::processEvents();
}
}  // namespace

TEST(HmiGuiTest, UpdateStatusTogglesButtons)
{
  std::vector<std::string> prefixes{"r1_"};
  std::vector<std::string> names{"R1"};
  HmiGui gui(prefixes, names);
  gui.show();
  pump();

  auto * start = gui.findChild<QPushButton *>("startButton");
  auto * stop = gui.findChild<QPushButton *>("stopButton");
  auto * reset = gui.findChild<QPushButton *>("resetButton");
  ASSERT_NE(start, nullptr);
  ASSERT_NE(stop, nullptr);
  ASSERT_NE(reset, nullptr);

  // stop_execution = true → start enabled, stop disabled
  gui.updateStatus(QString::fromStdString(prefixes[0]), true, false, false);
  pump();
  EXPECT_TRUE(start->isEnabled());
  EXPECT_FALSE(stop->isEnabled());
  EXPECT_TRUE(reset->isEnabled());

  // stop_execution = false → start disabled, stop enabled
  gui.updateStatus(QString::fromStdString(prefixes[0]), false, false, false);
  pump();
  EXPECT_FALSE(start->isEnabled());
  EXPECT_TRUE(stop->isEnabled());
  // reset disabled in this state per implementation
  EXPECT_FALSE(reset->isEnabled());
}

TEST(HmiGuiTest, UpdateRobotMessageSetsTextAndStyle)
{
  std::vector<std::string> prefixes{"r1_"};
  std::vector<std::string> names{"R1"};
  HmiGui gui(prefixes, names);
  gui.show();
  pump();

  gui.updateRobotMessage(QString::fromStdString(prefixes[0]), "Hello", "red");
  pump();

  // Find any label that matches the message
  bool found = false;
  for (auto * lbl : gui.findChildren<QLabel *>()) {
    if (lbl->text() == "Hello") {
      found = true;
      // Check that a style got applied
      EXPECT_FALSE(lbl->styleSheet().isEmpty());
      break;
    }
  }
  EXPECT_TRUE(found);
}

int main(int argc, char ** argv)
{
  setenv("QT_QPA_PLATFORM", "offscreen", 1);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
