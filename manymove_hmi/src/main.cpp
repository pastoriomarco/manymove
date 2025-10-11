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


#include <csignal>
#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include "manymove_hmi/hmi_gui.hpp"
#include "manymove_hmi/ros2_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <QDebug>
#include <string>
#include <vector>

using namespace std::chrono_literals;

void handleSigInt(int)
{
  qDebug() << "SIGINT received, quitting application.";
  QCoreApplication::quit();
}

int main(int argc, char * argv[])
{
  std::signal(SIGINT, handleSigInt);
  rclcpp::init(argc, argv);

  rclcpp::sleep_for(2000000000ns);

  QApplication app(argc, argv);
  app.setStyle(QStyleFactory::create("Fusion"));

  // Set up dark palette (unchanged)
  QPalette darkPalette;
  darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::WindowText, Qt::white);
  darkPalette.setColor(QPalette::Base, QColor(42, 42, 42));
  darkPalette.setColor(QPalette::AlternateBase, QColor(66, 66, 66));
  darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
  darkPalette.setColor(QPalette::ToolTipText, Qt::white);
  darkPalette.setColor(QPalette::Text, Qt::white);
  darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
  darkPalette.setColor(QPalette::ButtonText, Qt::white);
  darkPalette.setColor(QPalette::BrightText, Qt::red);
  darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  darkPalette.setColor(QPalette::HighlightedText, Qt::black);
  app.setPalette(darkPalette);

  // Set global stylesheet (unchanged)
  app.setStyleSheet(
    "QPushButton { font: bold 16px; padding: 10px; min-width: 120px; min-height: 50px; }"
    "QPushButton:disabled { background-color: grey; color: darkgrey; }"
    "QPushButton#startButton { background-color: green; color: black; }"
    "QPushButton#startButton:disabled { background-color: grey; color: darkgrey; }"
    "QPushButton#stopButton { background-color: red; color: black; }"
    "QPushButton#stopButton:disabled { background-color: grey; color: darkgrey; }"
    "QPushButton#resetButton { background-color: yellow; color: black; }"
    "QPushButton#resetButton:disabled { background-color: grey; color: darkgrey; }");

  // Define the main node to get robot prefixes parameter
  auto loader_node_hmi = rclcpp::Node::make_shared("loader_node_hmi");

  // Declare and get the parameter 'robot_prefixes' (default: {""})
  loader_node_hmi->declare_parameter<std::vector<std::string>>(
    "robot_prefixes",
    std::vector<std::string>({""}));

  std::vector<std::string> robot_prefixes;
  loader_node_hmi->get_parameter("robot_prefixes", robot_prefixes);

  // Declare and get the parameter 'robot_prefixes' (default: {""})
  loader_node_hmi->declare_parameter<std::vector<std::string>>(
    "robot_names",
    std::vector<std::string>({""}));

  std::vector<std::string> robot_names;
  loader_node_hmi->get_parameter("robot_names", robot_names);

  // Check if sizes match
  if (robot_prefixes.size() != robot_names.size()) {
    qCritical() << "Error: robot_prefixes and robot_names must have the same number of elements.";
    rclcpp::shutdown();
    return 1;
  }

  loader_node_hmi.reset();

  // Create the HMI GUI window with one row per robot.
  HmiGui gui(robot_prefixes, robot_names);
  gui.show();

  // Create one ROS2 worker per robot. We give each worker a unique node name that embeds its prefix.
  std::vector<std::shared_ptr<Ros2Worker>> workers;
  for (auto & prefix : robot_prefixes) {
    std::string node_name = prefix + "hmi_worker";
    auto worker = std::make_shared<Ros2Worker>(node_name, &gui, prefix);

    workers.push_back(worker);
  }

  // Connect the GUI signals to the workers.
  for (auto & worker : workers) {
    QObject::connect(
      &gui, &HmiGui::startExecutionRequested, [worker](const std::string & p)
      {
        if (worker->getRobotPrefix() == p) {
          worker->callStartExecution();
        }
      });
    QObject::connect(
      &gui, &HmiGui::stopExecutionRequested, [worker](const std::string & p)
      {
        if (worker->getRobotPrefix() == p) {
          worker->callStopExecution();
        }
      });
    QObject::connect(
      &gui, &HmiGui::resetProgramRequested, [worker](const std::string & p)
      {
        if (worker->getRobotPrefix() == p) {
          worker->callResetProgram();
        }
      });
  }

  // Spin each worker in its own thread.
  std::vector<std::thread> worker_threads;
  for (auto & w : workers) {
    worker_threads.emplace_back(
      [w]()
      {rclcpp::spin(w);});
  }

  int ret = app.exec();

  rclcpp::shutdown();
  for (auto & th : worker_threads) {
    if (th.joinable()) {
      th.join();
    }
  }
  return ret;
}
