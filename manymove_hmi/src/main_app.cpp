#include <csignal>
#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include "manymove_hmi/hmi_gui.hpp"
#include "manymove_hmi/app_module.hpp"
#include "manymove_hmi/ros2_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <QDebug>
#include <QVBoxLayout>

using namespace std::chrono_literals;

void handleSigInt(int)
{
    qDebug() << "SIGINT received, quitting application.";
    QCoreApplication::quit();
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, handleSigInt);
    rclcpp::init(argc, argv);
    
    // Give some time for ROS 2 to initialize.
    rclcpp::sleep_for(2000000000ns);
    
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));

    // Set dark palette (same as original main).
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

    app.setStyleSheet(
        "QPushButton { font: bold 16px; padding: 10px; min-width: 120px; min-height: 50px; }"
        "QPushButton:disabled { background-color: grey; color: darkgrey; }"
        "QPushButton#startButton { background-color: green; color: black; }"
        "QPushButton#startButton:disabled { background-color: grey; color: darkgrey; }"
        "QPushButton#stopButton { background-color: red; color: black; }"
        "QPushButton#stopButton:disabled { background-color: grey; color: darkgrey; }"
        "QPushButton#resetButton { background-color: yellow; color: black; }"
        "QPushButton#resetButton:disabled { background-color: grey; color: darkgrey; }");

    // Load robot prefixes parameter as before.
    auto loader_node_hmi = rclcpp::Node::make_shared("loader_node_hmi");
    loader_node_hmi->declare_parameter<std::vector<std::string>>(
        "robot_prefixes", std::vector<std::string>({""}));
    std::vector<std::string> prefixes;
    loader_node_hmi->get_parameter("robot_prefixes", prefixes);
    loader_node_hmi.reset();

    // Create the original HMI window (robot modules).
    HmiGui gui(prefixes);
    gui.show();

    // Append the application-specific module into the GUI.
    // We assume the HmiGui’s central widget has a QVBoxLayout.
    if (gui.centralWidget())
    {
        QVBoxLayout *mainLayout = qobject_cast<QVBoxLayout*>(gui.centralWidget()->layout());
        if (mainLayout)
        {
            AppModule *appModule = new AppModule(&gui);
            mainLayout->addWidget(appModule);

            // Example: connect the AppModule signal to a simple handler.
            QObject::connect(appModule, &AppModule::keyUpdateRequested,
                             [](const QString &key, const QString &value) {
                qDebug() << "AppModule key update:" << key << "=" << value;
                // Here you could invoke a ROS 2 service call to update the key in the blackboard.
            });
        }
        else
        {
            qDebug() << "Error: could not retrieve the main layout from HmiGui.";
        }
    }

    // Create ROS2 workers for each robot module (as before).
    std::vector<std::shared_ptr<Ros2Worker>> workers;
    for (auto &prefix : prefixes)
    {
        std::string node_name = prefix + "hmi_worker";
        auto worker = std::make_shared<Ros2Worker>(node_name, &gui, prefix);
        workers.push_back(worker);
    }
    for (auto &worker : workers)
    {
        QObject::connect(&gui, &HmiGui::startExecutionRequested, [worker](const std::string &p) {
            if (worker->getRobotPrefix() == p)
                worker->callStartExecution();
        });
        QObject::connect(&gui, &HmiGui::stopExecutionRequested, [worker](const std::string &p) {
            if (worker->getRobotPrefix() == p)
                worker->callStopExecution();
        });
        QObject::connect(&gui, &HmiGui::resetProgramRequested, [worker](const std::string &p) {
            if (worker->getRobotPrefix() == p)
                worker->callResetProgram();
        });
    }

    // Spin each worker in its own thread.
    std::vector<std::thread> worker_threads;
    for (auto &w : workers)
    {
        worker_threads.emplace_back([w](){ rclcpp::spin(w); });
    }

    int ret = app.exec();
    rclcpp::shutdown();
    for (auto &th : worker_threads)
    {
        if (th.joinable())
            th.join();
    }
    return ret;
}
