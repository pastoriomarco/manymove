#include "manymove_hmi/app_module.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDebug>
#include <QEvent>
#include <QPalette>

AppModule::AppModule(QWidget *parent) : QWidget(parent)
{
    layout_ = new QVBoxLayout(this);

    // Define key configurations.

    // tube_length: an editable double field.
    KeyConfig tubeLength;
    tubeLength.key = "tube_length";
    tubeLength.value_type = "double";
    tubeLength.editable = true;
    tubeLength.visible = true;
    tubeLength.computeFunction = [](const QMap<QString, QString>& values) -> QString {
        return values.value("tube_length", "");
    };

    // tube_diameter: an editable double field.
    KeyConfig tubeDiameter;
    tubeDiameter.key = "tube_diameter";
    tubeDiameter.value_type = "double";
    tubeDiameter.editable = true;
    tubeDiameter.visible = true;
    tubeDiameter.computeFunction = [](const QMap<QString, QString>& values) -> QString {
        return values.value("tube_diameter", "");
    };

    // tube_spawn_pose: computed from tube_length and tube_diameter.
    KeyConfig tubeSpawnPose;
    tubeSpawnPose.key = "tube_spawn_pose";
    tubeSpawnPose.value_type = "pose";
    tubeSpawnPose.editable = false;
    tubeSpawnPose.visible = false;
    tubeSpawnPose.computeFunction = [](const QMap<QString, QString>& values) -> QString {
        bool ok1 = false;
        double length = values.value("tube_length").toDouble(&ok1);
        if (!ok1) {
            return "";
        }
        double x = (length / 2.0) + 0.973 + 0.005;
        double y = -0.6465;
        double z = 0.8055;
        double roll = 1.57;
        double pitch = 2.05;
        double yaw = 1.57;
        QString poseJson = QString("{\"x\":%1,\"y\":%2,\"z\":%3,\"roll\":%4,\"pitch\":%5,\"yaw\":%6}")
                                .arg(x)
                                .arg(y)
                                .arg(z)
                                .arg(roll)
                                .arg(pitch)
                                .arg(yaw);
        return poseJson;
    };

    keyConfigs_.push_back(tubeLength);
    keyConfigs_.push_back(tubeDiameter);
    keyConfigs_.push_back(tubeSpawnPose);

    setupUI();
}

AppModule::~AppModule()
{
}

void AppModule::setupUI()
{
    // Create UI elements based on each key configuration.
    for (const auto &config : keyConfigs_) {
        // Create a container row widget.
        QWidget *rowWidget = new QWidget(this);
        QHBoxLayout *rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);

        // Create and add a label.
        QLabel *label = new QLabel(config.key, rowWidget);
        rowLayout->addWidget(label);

        if (config.editable) {
            // Create an editable QLineEdit.
            QLineEdit *lineEdit = new QLineEdit(rowWidget);
            // Initialize the stored value as empty and show "N/A" in grey.
            editableValues_[config.key] = "";
            lineEdit->setText("N/A");
            QPalette pal = lineEdit->palette();
            pal.setColor(QPalette::Text, Qt::gray);
            lineEdit->setPalette(pal);

            // Connect textChanged signal.
            connect(lineEdit, &QLineEdit::textChanged, this, &AppModule::onEditableFieldChanged);
            rowLayout->addWidget(lineEdit);
            keyWidgets_[config.key] = lineEdit;
            // Install event filter for focus events.
            lineEdit->installEventFilter(this);
        } else {
            // For computed fields, use a QLabel.
            QLabel *valueLabel = new QLabel("N/A", rowWidget);
            rowLayout->addWidget(valueLabel);
            keyWidgets_[config.key] = valueLabel;
        }
        layout_->addWidget(rowWidget);
        keyRowWidgets_[config.key] = rowWidget;
        rowWidget->setVisible(config.visible);
    }

    // Create the Send button.
    sendButton_ = new QPushButton("Send", this);
    layout_->addWidget(sendButton_);
    connect(sendButton_, &QPushButton::clicked, this, &AppModule::onSendButtonClicked);

    // Check the initial Send button state.
    updateSendButtonState();
}

void AppModule::onEditableFieldChanged(const QString &text)
{
    // Identify which QLineEdit sent the signal.
    QLineEdit *lineEdit = qobject_cast<QLineEdit*>(sender());
    if (!lineEdit)
        return;

    // Find the key associated with this QLineEdit.
    QString key;
    for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it) {
        if (it.value() == lineEdit) {
            key = it.key();
            break;
        }
    }
    if (key.isEmpty())
        return;

    // Update the internal stored value.
    if (text == "N/A")
        editableValues_[key].clear();
    else
        editableValues_[key] = text;

    // Validate immediately and update text color:
    // If non-empty (and not "N/A") but invalid, change text to red.
    bool valid = isFieldValid(key, text);
    QPalette pal = lineEdit->palette();
    if (!text.isEmpty() && text != "N/A" && !valid) {
        pal.setColor(QPalette::Text, Qt::red);
    } else {
        pal.setColor(QPalette::Text, Qt::white);
    }
    lineEdit->setPalette(pal);

    // Update computed fields and the Send button.
    updateComputedFields();
    updateSendButtonState();
}

void AppModule::updateComputedFields()
{
    // For computed (non-editable) keys, update the label text.
    for (const auto &config : keyConfigs_) {
        if (!config.editable) {
            QString computedValue = config.computeFunction(editableValues_);
            QLabel *label = qobject_cast<QLabel*>(keyWidgets_.value(config.key));
            if (label) {
                label->setText(computedValue.isEmpty() ? "N/A" : computedValue);
            }
        }
    }
}

void AppModule::updateSendButtonState()
{
    bool atLeastOneValid = false;
    bool hasInvalid = false;

    for (const auto &config : keyConfigs_) {
        if (!config.editable)
            continue;
        QString text = editableValues_.value(config.key);
        if (!text.isEmpty() && text != "N/A") {
            if (isFieldValid(config.key, text))
                atLeastOneValid = true;
            else {
                hasInvalid = true;
                break;
            }
        }
    }
    // Enable Send if at least one field is valid and none are invalid.
    sendButton_->setEnabled(!hasInvalid && atLeastOneValid);
}

bool AppModule::isFieldValid(const QString &key, const QString &text) const
{
    // If the text is empty or "N/A", treat it as not valid.
    if (text.isEmpty() || text == "N/A")
        return false;

    // For a double type, check if the text can be converted.
    for (const auto &config : keyConfigs_) {
        if (config.key == key) {
            if (config.value_type == "double") {
                bool ok = false;
                text.toDouble(&ok);
                return ok;
            }
            // Add other type validations as needed.
        }
    }
    return true;
}

void AppModule::onSendButtonClicked()
{
    // For each key, compute and emit the final value if valid.
    for (const auto &config : keyConfigs_) {
        QString finalValue;
        if (config.editable) {
            QString rawValue = editableValues_.value(config.key);
            if (rawValue.isEmpty() || !isFieldValid(config.key, rawValue))
                continue;
            finalValue = config.computeFunction(editableValues_);
            if (finalValue.isEmpty())
                continue;
        } else {
            QLabel *label = qobject_cast<QLabel*>(keyWidgets_.value(config.key));
            if (label) {
                finalValue = label->text();
                if (finalValue == "N/A" || finalValue.isEmpty())
                    continue;
            }
        }
        emit keyUpdateRequested(config.key, config.value_type, finalValue);
        qDebug() << "Sending key:" << config.key
                 << "Type:" << config.value_type
                 << "Value:" << finalValue;
    }
}

void AppModule::setKeyVisibility(const QString &key, bool visible)
{
    if (keyRowWidgets_.contains(key))
        keyRowWidgets_[key]->setVisible(visible);
}

void AppModule::updateField(const QString &key, const QString &newValue)
{
    if (!keyWidgets_.contains(key))
        return;

    QWidget *widget = keyWidgets_[key];
    if (QLineEdit *lineEdit = qobject_cast<QLineEdit*>(widget)) {
        if (newValue.isEmpty()) {
            editableValues_[key].clear();
            lineEdit->setText("N/A");
            QPalette pal = lineEdit->palette();
            pal.setColor(QPalette::Text, Qt::gray);
            lineEdit->setPalette(pal);
        } else {
            editableValues_[key] = newValue;
            lineEdit->setText(newValue);
            QPalette pal = lineEdit->palette();
            pal.setColor(QPalette::Text, Qt::gray);
            lineEdit->setPalette(pal);
        }
    } else if (QLabel *label = qobject_cast<QLabel*>(widget)) {
        label->setText(newValue.isEmpty() ? "N/A" : newValue);
    }
    updateComputedFields();
    updateSendButtonState();
}

bool AppModule::eventFilter(QObject *obj, QEvent *event)
{
    if (QLineEdit *lineEdit = qobject_cast<QLineEdit*>(obj)) {
        if (event->type() == QEvent::FocusIn) {
            // Clear "N/A" on focus in and switch text to white.
            if (lineEdit->text() == "N/A")
                lineEdit->clear();
            QPalette pal = lineEdit->palette();
            pal.setColor(QPalette::Text, Qt::white);
            lineEdit->setPalette(pal);
        } else if (event->type() == QEvent::FocusOut) {
            // If empty on focus out, revert to "N/A" in grey.
            if (lineEdit->text().isEmpty()) {
                lineEdit->setText("N/A");
                QPalette pal = lineEdit->palette();
                pal.setColor(QPalette::Text, Qt::gray);
                lineEdit->setPalette(pal);
            } else {
                // Validate on focus out and update text color accordingly.
                QString key;
                for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it) {
                    if (it.value() == lineEdit) {
                        key = it.key();
                        break;
                    }
                }
                if (!key.isEmpty()) {
                    bool valid = isFieldValid(key, lineEdit->text());
                    QPalette pal = lineEdit->palette();
                    pal.setColor(QPalette::Text, valid ? Qt::white : Qt::red);
                    lineEdit->setPalette(pal);
                }
            }
            updateSendButtonState();
        }
    }
    return QWidget::eventFilter(obj, event);
}
