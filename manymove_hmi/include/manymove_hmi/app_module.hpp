#ifndef APP_MODULE_HPP
#define APP_MODULE_HPP

#include <QWidget>
#include <QMap>
#include <QString>
#include <functional>
#include <vector>

class QLineEdit;
class QLabel;
class QPushButton;
class QVBoxLayout;
class QHBoxLayout;
class QEvent;

// Structure to configure each key in the module.
struct KeyConfig {
    QString key;             // Name of the key.
    QString value_type;      // Data type, e.g. "double", "pose", etc.
    bool editable;           // If true, an input field is shown; if false, the value is computed.
    bool visible;            // Controls the visibility of both the label and the field.
    // A lambda that receives the current editable key values and returns the final string to send.
    std::function<QString(const QMap<QString, QString>&)> computeFunction;
};

class AppModule : public QWidget
{
    Q_OBJECT
public:
    explicit AppModule(QWidget *parent = nullptr);
    ~AppModule();

public slots:
    // Toggle the visibility of a given key's row (label + widget).
    void setKeyVisibility(const QString &key, bool visible);

    // New public slot: allows external components to update a given key’s field.
    void updateField(const QString &key, const QString &newValue = QString());

signals:
    // Emitted when the user clicks Send.
    // key: the key name, value_type: type string, value: the computed or transformed value.
    void keyUpdateRequested(const QString &key, const QString &value_type, const QString &value);

private slots:
    void onSendButtonClicked();
    void onEditableFieldChanged(const QString &text);

private:
    // Sets up the UI based on key configurations.
    void setupUI();
    // Recalculate and update computed fields.
    void updateComputedFields();
    // Check validations across all fields and update the Send button accordingly.
    void updateSendButtonState();
    // Returns true if the text is valid for the given key.
    bool isFieldValid(const QString &key, const QString &text) const;

protected:
    // Event filter to handle focus events for QLineEdits.
    bool eventFilter(QObject *obj, QEvent *event) override;

    // List of key configurations.
    std::vector<KeyConfig> keyConfigs_;
    // For each key, store its editable/computed widget (QLineEdit or QLabel).
    QMap<QString, QWidget*> keyWidgets_;
    // For each key, store the container widget (row) that includes the label and the widget.
    QMap<QString, QWidget*> keyRowWidgets_;
    // For editable keys, store the current text values.
    QMap<QString, QString> editableValues_;

    QPushButton *sendButton_;
    QVBoxLayout *layout_;
};

#endif // APP_MODULE_HPP
