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

#ifndef HMI_GUI_HPP
#define HMI_GUI_HPP

#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QPushButton>
#include <QLabel>
#include <QString>
#include <vector>
#include <string>

class HmiGui : public QMainWindow
{
Q_OBJECT

public:
/// The new constructor accepts a list of robot prefixes.
explicit HmiGui(const std::vector<std::string>& robotPrefixes,
                std::vector<std::string>& robotNames, QWidget* parent = nullptr);
~HmiGui();

public slots:
/// Update the GUI for a given robot (by prefix).
void updateStatus(const QString& robotPrefix,
                  bool stop_execution,
                  bool reset,
                  bool collision_detected);

/// Update a textual message for a given robot.
void updateRobotMessage(const QString& robotPrefix,
                        const QString& message,
                        const QString& color);

// TCP server slots
void onNewConnection();
void onSocketDisconnected();

signals:
/// Signals that include the robot prefix.
void startExecutionRequested(const std::string& robotPrefix);
void stopExecutionRequested(const std::string& robotPrefix);
void resetProgramRequested(const std::string& robotPrefix);

private:
QWidget* centralWidget_;
QTcpServer* tcpServer_;
QTcpSocket* clientSocket_;

// Structure holding one robot’s UI elements.
struct RobotInterface
{
  std::string prefix;
  QLabel* prefixLabel;
  QPushButton* startButton;
  QPushButton* stopButton;
  QPushButton* resetButton;
  QLabel* messageLabel;
};

std::vector<RobotInterface> robotInterfaces_;
std::vector<std::string> robotNames_;

// (Optional) For TCP status broadcasting.
QString lastStatusJson_;
};

#endif // HMI_GUI_HPP
