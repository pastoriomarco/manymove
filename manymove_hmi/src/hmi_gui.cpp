#include "manymove_hmi/hmi_gui.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>
#include <QMetaType>
#include <sstream>

HmiGui::HmiGui(const std::vector<std::string> &robotPrefixes, QWidget *parent)
    : QMainWindow(parent), tcpServer_(nullptr), clientSocket_(nullptr)
{
    // Keep the window always on top
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

    // Create the central widget + main vertical layout
    centralWidget_ = new QWidget(this);
    setCentralWidget(centralWidget_);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget_);

    // For each robot prefix, create one row of controls
    for (const auto &prefix : robotPrefixes)
    {
        QHBoxLayout *rowLayout = new QHBoxLayout();

        // A label to display the prefix name
        QLabel *prefixLabel = new QLabel(QString::fromStdString(prefix), this);
        rowLayout->addWidget(prefixLabel);

        // START button
        QPushButton *startButton = new QPushButton("START", this);
        startButton->setObjectName("startButton");
        startButton->setEnabled(true);
        rowLayout->addWidget(startButton);

        // STOP button
        QPushButton *stopButton = new QPushButton("STOP", this);
        stopButton->setObjectName("stopButton");
        stopButton->setEnabled(false);
        rowLayout->addWidget(stopButton);

        // RESET button
        QPushButton *resetButton = new QPushButton("RESET", this);
        resetButton->setObjectName("resetButton");
        resetButton->setEnabled(true);
        rowLayout->addWidget(resetButton);

        // Collision label & LED
        rowLayout->addWidget(new QLabel("Collision:", this));
        QLabel *ledIndicator = new QLabel(this);
        ledIndicator->setFixedSize(20, 20);
        ledIndicator->setStyleSheet("background-color: green; border-radius: 10px; border: 1px solid darkgreen;");
        rowLayout->addWidget(ledIndicator);

        // Add the row to the main layout
        mainLayout->addLayout(rowLayout);

        // Store references in the RobotInterface struct
        RobotInterface ri;
        ri.prefix = prefix;
        ri.prefixLabel = prefixLabel;
        ri.startButton = startButton;
        ri.stopButton = stopButton;
        ri.resetButton = resetButton;
        ri.ledIndicator = ledIndicator;
        robotInterfaces_.push_back(ri);

        // Wire each button's click to a signal with the robot prefix
        connect(startButton, &QPushButton::clicked, this, [this, prefix]()
                { emit startExecutionRequested(prefix); });
        connect(stopButton, &QPushButton::clicked, this, [this, prefix]()
                { emit stopExecutionRequested(prefix); });
        connect(resetButton, &QPushButton::clicked, this, [this, prefix]()
                { emit resetProgramRequested(prefix); });
    }

    // Set up a TCP server listening on port 5000
    tcpServer_ = new QTcpServer(this);
    connect(tcpServer_, &QTcpServer::newConnection, this, &HmiGui::onNewConnection);
    if (!tcpServer_->listen(QHostAddress::Any, 5000))
    {
        qWarning() << "TCP Server could not start!";
    }
    else
    {
        qDebug() << "TCP Server listening on port 5000";
    }
}

HmiGui::~HmiGui()
{
    if (clientSocket_)
    {
        clientSocket_->disconnectFromHost();
    }
}

void HmiGui::updateStatus(const QString &robotPrefix,
                          bool stop_execution,
                          bool reset,
                          bool collision_detected)
{
    // Find the row for this robotPrefix
    for (auto &ri : robotInterfaces_)
    {
        if (ri.prefix == robotPrefix.toStdString())
        {
            // Update button states
            if (stop_execution)
            {
                ri.startButton->setEnabled(true);
                ri.resetButton->setEnabled(true);
                ri.stopButton->setEnabled(false);
            }
            else
            {
                ri.startButton->setEnabled(false);
                ri.resetButton->setEnabled(false);
                ri.stopButton->setEnabled(true);
            }

            // Collision LED
            if (collision_detected)
            {
                ri.ledIndicator->setStyleSheet("background-color: red; border-radius: 10px; border: 1px solid darkred;");
            }
            else
            {
                ri.ledIndicator->setStyleSheet("background-color: green; border-radius: 10px; border: 1px solid darkgreen;");
            }

            // Build a JSON string for status (if you want to send it via TCP)
            std::ostringstream ss;
            ss << "{\"" << ri.prefix << "stop_execution\": " << (stop_execution ? "true" : "false")
               << ", \"" << ri.prefix << "reset\": " << (reset ? "true" : "false")
               << ", \"" << ri.prefix << "collision_detected\": " << (collision_detected ? "true" : "false")
               << "}";

            lastStatusJson_ = QString::fromStdString(ss.str());
            break;
        }
    }

    // If a TCP client is connected, send the updated status
    if (clientSocket_ && clientSocket_->state() == QAbstractSocket::ConnectedState)
    {
        clientSocket_->write(lastStatusJson_.toUtf8());
        clientSocket_->flush();
    }
}

void HmiGui::onNewConnection()
{
    clientSocket_ = tcpServer_->nextPendingConnection();
    connect(clientSocket_, &QTcpSocket::disconnected, this, &HmiGui::onSocketDisconnected);
    qDebug() << "New TCP client connected.";
    if (!lastStatusJson_.isEmpty())
    {
        clientSocket_->write(lastStatusJson_.toUtf8());
        clientSocket_->flush();
    }
}

void HmiGui::onSocketDisconnected()
{
    qDebug() << "TCP client disconnected.";
    clientSocket_ = nullptr;
}
