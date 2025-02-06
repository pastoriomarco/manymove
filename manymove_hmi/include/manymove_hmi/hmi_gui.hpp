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
    explicit HmiGui(const std::vector<std::string> &robotPrefixes, QWidget *parent = nullptr);
    ~HmiGui();

public slots:
    /// Update the GUI for a given robot (by prefix).
    void updateStatus(const QString &robotPrefix,
                      bool execution_resumed,
                      bool stop_execution,
                      bool abort_mission,
                      bool collision_detected);

    // TCP server slots (unchanged)
    void onNewConnection();
    void onSocketDisconnected();

signals:
    /// Signals that include the robot prefix.
    void startExecutionRequested(const std::string &robotPrefix);
    void stopExecutionRequested(const std::string &robotPrefix);
    void resetProgramRequested(const std::string &robotPrefix);

private:
    QWidget *centralWidget_;
    QTcpServer *tcpServer_;
    QTcpSocket *clientSocket_;

    // Structure holding one robot’s UI elements.
    struct RobotInterface
    {
        std::string prefix;
        QLabel *prefixLabel;
        QPushButton *startButton;
        QPushButton *stopButton;
        QPushButton *resetButton;
        QLabel *ledIndicator;
    };

    std::vector<RobotInterface> robotInterfaces_;

    // (Optional) For TCP status broadcasting.
    QString lastStatusJson_;
};

#endif // HMI_GUI_HPP
