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
#include <QLineEdit>
#include <QPushButton>
#include <QToolButton>
#include <chrono>
#include <cstdlib>
#include <string>
#include <tuple>
#include <vector>

#include "manymove_hmi/app_module.hpp"

namespace
{
struct Emission
{
  QString key;
  QString type;
  QString value;
};

// Helper to process the Qt event loop briefly
void pump() {QCoreApplication::processEvents();}
}  // namespace

TEST(AppModuleTest, CycleButtonEmitsOnToggle)
{
  // Minimal config: only the cycle toggle
  std::vector<KeyConfig> keys;
  keys.emplace_back(
    KeyConfig{"cycle_on_key", "bool", true, true, [](const QMap<QString, QString> & v)
      {return v.value("cycle_on_key", "");}});

  AppModule module(keys);
  module.show();
  pump();

  std::vector<Emission> emissions;
  QObject::connect(
    &module, &AppModule::keyUpdateRequested,
    [&](const QString & k, const QString & t, const QString & v) {
      emissions.push_back({k, t, v});
    });

  // Find the only QToolButton (cycle button)
  auto buttons = module.findChildren<QToolButton *>();
  ASSERT_EQ(buttons.size(), 1);
  auto * cycle = buttons.front();
  ASSERT_STREQ(cycle->text().toUtf8().constData(), "ROBOT CYCLE OFF");

  // Toggle ON -> emits true
  cycle->setChecked(true);
  pump();
  ASSERT_FALSE(emissions.empty());
  EXPECT_EQ(emissions.back().key, QString("cycle_on_key"));
  EXPECT_EQ(emissions.back().type, QString("bool"));
  EXPECT_EQ(emissions.back().value, QString("true"));

  // Toggle OFF -> emits false
  cycle->setChecked(false);
  pump();
  ASSERT_GE(emissions.size(), 2u);
  EXPECT_EQ(emissions.back().value, QString("false"));
}

TEST(AppModuleTest, EditValidateSendAndNormalization)
{
  // Keys: cycle + one editable double with scale and limits
  std::vector<KeyConfig> keys;
  keys.emplace_back(
    KeyConfig{"cycle_on_key", "bool", true, true, [](const QMap<QString, QString> & v)
      {return v.value("cycle_on_key", "");}});
  keys.emplace_back(
    KeyConfig{"tube_length_key", "double", true, true, [](const QMap<QString, QString> & v)
      {return v.value("tube_length_key", "");}, 1000.0, "mm", true, 500, 47.0, 150.0});

  AppModule module(keys);
  module.show();
  pump();

  // Grab widgets
  auto edits = module.findChildren<QLineEdit *>();
  ASSERT_EQ(edits.size(), 1);
  auto * edit = edits.front();

  QPushButton * sendBtn = nullptr;
  for (auto * b : module.findChildren<QPushButton *>()) {
    if (b->text() == "SEND") {
      sendBtn = b;
      break;
    }
  }
  ASSERT_NE(sendBtn, nullptr);

  // Initially SEND disabled
  EXPECT_FALSE(sendBtn->isEnabled());

  // Enter a valid value in mm (100 mm => internal 0.1)
  edit->setText("100");
  pump();
  EXPECT_TRUE(sendBtn->isEnabled());

  std::vector<Emission> emissions;
  QObject::connect(
    &module, &AppModule::keyUpdateRequested,
    [&](const QString & k, const QString & t, const QString & v) {
      emissions.push_back({k, t, v});
    });

  // Click SEND -> should emit update for tube_length_key with normalized internal value
  sendBtn->click();
  pump();

  ASSERT_FALSE(emissions.empty());
  bool found = false;
  for (const auto & e : emissions) {
    if (e.key == "tube_length_key") {
      found = true;
      EXPECT_EQ(e.type, QString("double"));
      EXPECT_EQ(e.value, QString("0.1"));
    }
  }
  EXPECT_TRUE(found);

  // After sending, overrides cleared and SEND disabled again
  EXPECT_FALSE(sendBtn->isEnabled());

  // Now set below lower limit (46 mm => 0.046) → SEND disabled
  edit->setText("46");
  pump();
  EXPECT_FALSE(sendBtn->isEnabled());

  // Clear text and force focus-out so AppModule clears overrides
  edit->setText("");
  pump();
  sendBtn->setFocus();  // triggers FocusOut for edit via eventFilter
  pump();

  // updateField should normalize numeric and update display with scale
  module.updateField("tube_length_key", "0.2");  // internal units
  pump();
  EXPECT_EQ(edit->text(), QString("200"));  // displayed in mm
}

int main(int argc, char ** argv)
{
  setenv("QT_QPA_PLATFORM", "offscreen", 1);
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
