#ifndef APP_MODULE_HPP
#define APP_MODULE_HPP

#include <QWidget>
#include <QMap>
#include <QString>
#include <functional>
#include <vector>
#include <limits>
#include <cmath>

class QLineEdit;
class QLabel;
class QPushButton;
class QVBoxLayout;
class QHBoxLayout;
class QEvent;

/* ------------------------------------------------------------------ */
/*  Generic descriptors                                               */
/* ------------------------------------------------------------------ */

struct KeyConfig // how one line of the HMI behaves
{
    QString key;           // black-board key name
    QString value_type;    // bool | double | int | pose | …
    bool editable = false; // user can type?
    bool visible = true;   // row shown?
    std::function<QString(const QMap<QString, QString> &)> computeFunction;
    double display_scale = 1.0; // multiply by this for GUI
    QString unit;               // unit label shown in GUI
    bool show_label = true;     // display the key label?
    int widget_width = 500;     // width of the input/control widget
    double lower_limit = std::numeric_limits<double>::quiet_NaN();
    double upper_limit = std::numeric_limits<double>::quiet_NaN();

    KeyConfig(const QString &k,
              const QString &type,
              bool ed,
              bool vis,
              std::function<QString(const QMap<QString, QString> &)> fn = {},
              double scale = 1.0,
              const QString &u = QString(),
              bool show_lbl = true,
              int width = 500,
              double lower = std::numeric_limits<double>::quiet_NaN(),
              double upper = std::numeric_limits<double>::quiet_NaN())
        : key(k), value_type(type), editable(ed), visible(vis), computeFunction(std::move(fn)),
          display_scale(scale), unit(u), show_label(show_lbl), widget_width(width)
    {
        lower_limit = std::isnan(lower) ? lower : lower / scale;
        upper_limit = std::isnan(upper) ? upper : upper / scale;
    }
};

struct BlackboardKey // minimal info used by Ros2Worker
{
    QString key;
    QString type;
};

enum class FieldStatus
{
    Valid,
    Invalid,
    BelowLimit,
    AboveLimit
};

/* ------------------------------------------------------------------ */
/*  Generic HMI engine                                                */
/* ------------------------------------------------------------------ */

class AppModule : public QWidget
{
    Q_OBJECT
public:
    ///  Construct from a *list* of KeyConfig. Nothing is hard-coded.
    explicit AppModule(const std::vector<KeyConfig> &key_cfg,
                       QWidget *parent = nullptr);
    ~AppModule() override = default;

    ///  Keys understood by this instance – Ros2Worker uses this.
    const std::vector<BlackboardKey> &getKnownKeys() const { return bb_keys_; }

public slots:
    void setKeyVisibility(const QString &key, bool visible);
    void updateField(const QString &key,
                     const QString &newValue = QString());
    void updateGeneralMessage(const QString &message,
                              const QString &color);

signals:
    void keyUpdateRequested(const QString &key,
                            const QString &value_type,
                            const QString &value);

private slots:
    void onSendClicked();
    void onEditableChanged(const QString &text);

private:
    /* helpers ------------------------------------------------------- */
    void setupUI();
    void updateComputedFields();
    void updateSendButtonState();
    bool isFieldValid(const QString &key, const QString &text) const;
    FieldStatus validateField(const QString &key, const QString &text) const;
    QString toDisplay(const QString &key, const QString &val) const;
    QString toInternal(const QString &key, const QString &val) const;

    /* QWidget override --------------------------------------------- */
    bool eventFilter(QObject *obj, QEvent *event) override;

protected:
    /* configuration & runtime state -------------------------------- */
    std::vector<KeyConfig> keyConfigs_;
    std::vector<BlackboardKey> bb_keys_; // derived once

    QMap<QString, QWidget *> keyWidgets_;
    QMap<QString, QWidget *> keyRowWidgets_;
    QMap<QString, QString> currentValues_;
    QMap<QString, QString> userOverrides_;
    QMap<QString, QString> editableValues_;

    QPushButton *sendButton_{nullptr};
    QVBoxLayout *layout_{nullptr};
    QLabel *generalMessage_{nullptr};
};

#endif /* APP_MODULE_HPP */
