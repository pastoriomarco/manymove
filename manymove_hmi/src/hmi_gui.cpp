/*
 * BSD 3-Clause License
 * Copyright (c) 2024-2025, Flexin Group SRL
 * All rights reserved.
 *
 * See the LICENSE file in the project root for full license information.
 */

#include "manymove_hmi/hmi_gui.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>
#include <QMetaType>
#include <QPalette>
#include <QColor>
#include <sstream>

HmiGui::HmiGui(
  const std::vector<std::string> & robotPrefixes,
  std::vector<std::string> & robotNames, QWidget * parent)
: QMainWindow(parent), tcpServer_(nullptr), clientSocket_(nullptr)
{
  // Keep the window always on top
  setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

  // Create the central widget + main vertical layout
  centralWidget_ = new QWidget(this);
  setCentralWidget(centralWidget_);
  QVBoxLayout * mainLayout = new QVBoxLayout(centralWidget_);

  // For each robot prefix, create one row of controls
  for (size_t i{0}; i < robotPrefixes.size(); i++) {
    std::string prefix = robotPrefixes[i];
    std::string name = robotNames[i];

    QVBoxLayout * robotLayout = new QVBoxLayout();

    // A label to display the prefix name
    QLabel * prefixLabel = new QLabel(QString::fromStdString("ROBOT: " + prefix + name), this);
    robotLayout->addWidget(prefixLabel);

    QHBoxLayout * rowLayout = new QHBoxLayout();

    // START button
    QPushButton * startButton = new QPushButton("START", this);
    startButton->setObjectName("startButton");
    startButton->setEnabled(true);
    rowLayout->addWidget(startButton);

    // STOP button
    QPushButton * stopButton = new QPushButton("STOP", this);
    stopButton->setObjectName("stopButton");
    stopButton->setEnabled(false);
    rowLayout->addWidget(stopButton);

    // RESET button
    QPushButton * resetButton = new QPushButton("RESET", this);
    resetButton->setObjectName("resetButton");
    resetButton->setEnabled(true);
    rowLayout->addWidget(resetButton);

    // Add the row to the robot layout
    robotLayout->addLayout(rowLayout);

    QLabel * msgLbl = new QLabel(this);
    msgLbl->setFixedHeight(50);
    QFont msgFont = msgLbl->font();
    msgFont.setPointSize(14);
    msgLbl->setFont(msgFont);
    robotLayout->addWidget(msgLbl);

    // Add the robot layout to the main layout
    mainLayout->addLayout(robotLayout);

    // Store references in the RobotInterface struct
    RobotInterface ri;
    ri.prefix = prefix;
    ri.prefixLabel = prefixLabel;
    ri.startButton = startButton;
    ri.stopButton = stopButton;
    ri.resetButton = resetButton;
    ri.messageLabel = msgLbl;
    robotInterfaces_.push_back(ri);

    // Wire each button's click to a signal with the robot prefix
    connect(
      startButton, &QPushButton::clicked, this, [this, prefix]()
      {emit startExecutionRequested(prefix);});
    connect(
      stopButton, &QPushButton::clicked, this, [this, prefix]()
      {emit stopExecutionRequested(prefix);});
    connect(
      resetButton, &QPushButton::clicked, this, [this, prefix]()
      {emit resetProgramRequested(prefix);});
  }

  // Set up a TCP server listening on port 5000
  tcpServer_ = new QTcpServer(this);
  connect(tcpServer_, &QTcpServer::newConnection, this, &HmiGui::onNewConnection);
  if (!tcpServer_->listen(QHostAddress::Any, 5000)) {
    qWarning() << "TCP Server could not start!";
  } else {
    qDebug() << "TCP Server listening on port 5000";
  }
}

HmiGui::~HmiGui()
{
  if (clientSocket_) {
    clientSocket_->disconnectFromHost();
  }
}

void HmiGui::updateStatus(
  const QString & robotPrefix,
  bool stop_execution,
  bool reset,
  bool collision_detected)
{
  // Find the row for this robotPrefix
  for (auto & ri : robotInterfaces_) {
    if (ri.prefix == robotPrefix.toStdString()) {
      // Update button states
      if (stop_execution) {
        ri.startButton->setEnabled(true);
        ri.resetButton->setEnabled(true);
        ri.stopButton->setEnabled(false);
      } else {
        ri.startButton->setEnabled(false);
        ri.resetButton->setEnabled(false);
        ri.stopButton->setEnabled(true);
      }

      // Build a JSON string for status (if you want to send it via TCP)
      std::ostringstream ss;
      ss << "{\"" << ri.prefix << "stop_execution\": " << (stop_execution ? "true" : "false")
         << ", \"" << ri.prefix << "reset\": " << (reset ? "true" : "false")
         << ", \"" << ri.prefix << "collision_detected\": " <<
        (collision_detected ? "true" : "false")
         << "}";

      lastStatusJson_ = QString::fromStdString(ss.str());
      break;
    }
  }

  // If a TCP client is connected, send the updated status
  if (clientSocket_ && clientSocket_->state() == QAbstractSocket::ConnectedState) {
    clientSocket_->write(lastStatusJson_.toUtf8());
    clientSocket_->flush();
  }
}

void HmiGui::updateRobotMessage(
  const QString & robotPrefix,
  const QString & message,
  const QString & color)
{
  for (auto & ri : robotInterfaces_) {
    if (ri.prefix == robotPrefix.toStdString()) {
      ri.messageLabel->setText(message);
      if (!color.isEmpty()) {
        ri.messageLabel->setStyleSheet(
          QString("color: %1; border: 2px solid %1; padding:2px;").arg(
            color));
      } else {
        ri.messageLabel->setStyleSheet("");
      }
      break;
    }
  }
}

void HmiGui::onNewConnection()
{
  clientSocket_ = tcpServer_->nextPendingConnection();
  connect(clientSocket_, &QTcpSocket::disconnected, this, &HmiGui::onSocketDisconnected);
  qDebug() << "New TCP client connected.";
  if (!lastStatusJson_.isEmpty()) {
    clientSocket_->write(lastStatusJson_.toUtf8());
    clientSocket_->flush();
  }
}

void HmiGui::onSocketDisconnected()
{
  qDebug() << "TCP client disconnected.";
  clientSocket_ = nullptr;
}
