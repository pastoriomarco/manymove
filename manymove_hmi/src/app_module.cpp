#include "manymove_hmi/app_module.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDebug>
#include <QEvent>
#include <QPalette>

// Static function returning the known keys.
// This vector is constructed only once.
const std::vector<BlackboardKey> &AppModule::getKnownKeys()
{
    static std::vector<BlackboardKey> keys = {
        {"tube_length", "double"},
        {"tube_diameter", "double"},
        {"tube_scale", "double_array"},
        {"tube_spawn_pose", "pose"},
    };
    return keys;
}

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
    tubeLength.computeFunction = [](const QMap<QString, QString> &values) -> QString
    {
        return values.value("tube_length", "");
    };

    // tube_diameter: an editable double field.
    KeyConfig tubeDiameter;
    tubeDiameter.key = "tube_diameter";
    tubeDiameter.value_type = "double";
    tubeDiameter.editable = true;
    tubeDiameter.visible = true;
    tubeDiameter.computeFunction = [](const QMap<QString, QString> &values) -> QString
    {
        return values.value("tube_diameter", "");
    };

    // tube_scale: an editable double_array field.
    KeyConfig tubeScale;
    tubeScale.key = "tube_scale";
    tubeScale.value_type = "double_array";
    tubeScale.editable = false;
    tubeScale.visible = true;
    tubeScale.computeFunction = [this](const QMap<QString, QString> &values) -> QString
    {
        // Get the tube_length value: check first in the override map, then in currentValues_ as fallback.
        QString tubeLengthStr = values.value("tube_length", "");
        // Get the tube_diameter value similarly.
        QString tubeDiameterStr = values.value("tube_diameter", "");

        // If either value is still empty, return an empty string so that no computed update overwrites the current value.
        if (tubeLengthStr.isEmpty() && tubeDiameterStr.isEmpty())
        {
            return "";
        }

        if (tubeLengthStr.isEmpty())
        {
            tubeLengthStr = currentValues_.value("tube_length", "");
        }
        if (tubeDiameterStr.isEmpty())
        {
            tubeDiameterStr = currentValues_.value("tube_diameter", "");
        }

        bool ok1 = false, ok2 = false;
        double tubeLengthVal = tubeLengthStr.toDouble(&ok1);
        double tubeDiameterVal = tubeDiameterStr.toDouble(&ok2);
        if (!ok1 || !ok2)
        {
            return "";
        }

        // Construct the computed double array string.
        // Here, both the first two values are set to the tube_diameter (from input or fallback)
        // and the third value is set from the tube_length.
        QString computedArray = QString("[%1, %2, %3]")
                                    .arg(tubeDiameterVal)
                                    .arg(tubeDiameterVal)
                                    .arg(tubeLengthVal);
        return computedArray;
    };

    // tube_spawn_pose: computed from tube_length and tube_diameter.
    KeyConfig tubeSpawnPose;
    tubeSpawnPose.key = "tube_spawn_pose";
    tubeSpawnPose.value_type = "pose";
    tubeSpawnPose.editable = false;
    tubeSpawnPose.visible = true;
    tubeSpawnPose.computeFunction = [](const QMap<QString, QString> &values) -> QString
    {
        // If the user did not override tube_length, return empty.
        // (Assume that an override means the user has typed a new value)
        QString tubeLengthOverride = values.value("tube_length", "");
        if (tubeLengthOverride.isEmpty())
        {
            // No computed update was triggered: return empty so that the current (ROS) value is used.
            return "";
        }

        bool ok = false;
        double length = tubeLengthOverride.toDouble(&ok);
        if (!ok)
            return "";

        // Compute a new pose based on the new tube_length.
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
    keyConfigs_.push_back(tubeScale);
    keyConfigs_.push_back(tubeSpawnPose);

    // Initialize maps for each key.
    for (const auto &config : keyConfigs_)
    {
        currentValues_[config.key] = "N/A"; // Initially, the current value is "N/A"
        userOverrides_[config.key] = "";    // No user override yet.
        editableValues_[config.key] = "";   // Legacy storage remains.
    }

    setupUI();
}

AppModule::~AppModule()
{
}

void AppModule::setupUI()
{
    // Create UI elements based on each key configuration.
    for (const auto &config : keyConfigs_)
    {
        // Create a container row widget.
        QWidget *rowWidget = new QWidget(this);
        QHBoxLayout *rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);

        // Create and add a label.
        QLabel *label = new QLabel(config.key, rowWidget);
        label->setFixedWidth(200);
        rowLayout->addWidget(label);

        if (config.editable)
        {
            // Create an editable QLineEdit.
            QLineEdit *lineEdit = new QLineEdit(rowWidget);
            lineEdit->setAlignment(Qt::AlignRight);
            lineEdit->setFixedWidth(750);
            // Initialize to show the current value ("N/A") in grey.
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
        }
        else
        {
            // For computed fields, use a QLabel.
            QLabel *valueLabel = new QLabel("N/A", rowWidget);
            valueLabel->setAlignment(Qt::AlignRight);
            valueLabel->setFixedWidth(750);
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
    QLineEdit *lineEdit = qobject_cast<QLineEdit *>(sender());
    if (!lineEdit)
        return;

    // Find the key associated with this QLineEdit.
    QString key;
    for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it)
    {
        if (it.value() == lineEdit)
        {
            key = it.key();
            break;
        }
    }
    if (key.isEmpty())
        return;

    // If the user input equals the current value, treat it as empty.
    if (text == currentValues_[key])
    {
        userOverrides_[key].clear();
        editableValues_[key].clear();
    }
    else
    {
        userOverrides_[key] = text;
        editableValues_[key] = text;
    }

    // Validate immediately and update text color:
    bool valid = isFieldValid(key, text);
    QPalette pal = lineEdit->palette();
    if (!text.isEmpty() && text != "N/A" && !valid)
    {
        pal.setColor(QPalette::Text, Qt::red);
    }
    else
    {
        pal.setColor(QPalette::Text, Qt::white);
    }
    lineEdit->setPalette(pal);

    updateComputedFields();
    updateSendButtonState();
}

void AppModule::updateComputedFields()
{
    // For each computed (noneditable) key, update the label text.
    for (const auto &config : keyConfigs_)
    {
        if (!config.editable)
        {
            QLabel *label = qobject_cast<QLabel *>(keyWidgets_.value(config.key));
            if (!label)
                continue;

            // Call the compute function.
            QString computedValue = config.computeFunction(editableValues_);
            QPalette pal = label->palette();
            if (!computedValue.isEmpty())
            {
                // A computed value is available. Use it and set text color to white.
                currentValues_[config.key] = computedValue;
                label->setText(computedValue);
                pal.setColor(QPalette::WindowText, Qt::white);
                label->setPalette(pal);
            }
            else
            {
                // No computed value; fall back to the current (ROS-provided) value.
                label->setText(currentValues_.value(config.key, "N/A"));
                pal.setColor(QPalette::WindowText, Qt::gray);
                label->setPalette(pal);
            }
        }
    }
}

void AppModule::updateSendButtonState()
{
    bool atLeastOneValid = false;
    bool hasInvalid = false;

    for (const auto &config : keyConfigs_)
    {
        if (!config.editable)
            continue;
        // Use userOverrides_; if override is empty or equals current, treat as empty.
        QString text = userOverrides_.value(config.key);
        if (text.isEmpty() || text == currentValues_[config.key])
            continue;
        if (isFieldValid(config.key, text))
            atLeastOneValid = true;
        else
        {
            hasInvalid = true;
            break;
        }
    }
    // Enable Send if at least one field is valid and none are invalid.
    sendButton_->setEnabled(!hasInvalid && atLeastOneValid);
}

bool AppModule::isFieldValid(const QString &key, const QString &text) const
{
    // Treat empty, "N/A", or a value equal to the current value as not valid (i.e. no change).
    if (text.isEmpty() || text == "N/A" || text == currentValues_.value(key))
        return false;

    for (const auto &config : keyConfigs_)
    {
        if (config.key == key)
        {
            if (config.value_type == "double")
            {
                bool ok = false;
                text.toDouble(&ok);
                return ok;
            }
            // Additional type validations can be added here.
        }
    }
    return true;
}

void AppModule::onSendButtonClicked()
{
    // For each key, compute and emit the final value if valid.
    for (const auto &config : keyConfigs_)
    {
        QString finalValue;
        if (config.editable)
        {
            QString rawValue = userOverrides_.value(config.key);
            // Only send if override is non-empty and different from current.
            if (rawValue.isEmpty() || rawValue == currentValues_[config.key] || !isFieldValid(config.key, rawValue))
                continue;
            finalValue = config.computeFunction(editableValues_);
            if (finalValue.isEmpty())
                continue;
        }
        else
        {
            QLabel *label = qobject_cast<QLabel *>(keyWidgets_.value(config.key));
            if (label)
            {
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
    // Once Send is pressed, clear all user overrides so that the latest current values are shown.
    for (const auto &config : keyConfigs_)
    {
        if (config.editable)
        {
            userOverrides_[config.key].clear();
            // Reset the field to the current value (displayed in grey).
            if (QLineEdit *lineEdit = qobject_cast<QLineEdit *>(keyWidgets_[config.key]))
            {
                lineEdit->setText(currentValues_[config.key]);
                QPalette pal = lineEdit->palette();
                pal.setColor(QPalette::Text, Qt::gray);
                lineEdit->setPalette(pal);
            }
        }
    }
    updateComputedFields();
    updateSendButtonState();
}

void AppModule::setKeyVisibility(const QString &key, bool visible)
{
    if (keyRowWidgets_.contains(key))
        keyRowWidgets_[key]->setVisible(visible);
}

// This function is called externally to update the "current" (blackboard) value.
// It will update the field only if the user is not currently overriding it.
void AppModule::updateField(const QString &key, const QString &newValue)
{
    if (!keyWidgets_.contains(key))
        return;

    QWidget *widget = keyWidgets_[key];
    // Update the stored current value.
    currentValues_[key] = newValue.isEmpty() ? "N/A" : newValue;

    if (QLineEdit *lineEdit = qobject_cast<QLineEdit *>(widget))
    {
        // If the field has focus or a user override exists (non-empty and different from current), do not update.
        if (lineEdit->hasFocus() || (!userOverrides_[key].isEmpty() && userOverrides_[key] != currentValues_[key]))
        {
            return;
        }
        // Otherwise, update the field with the current value (in grey).
        lineEdit->setText(currentValues_[key]);
        QPalette pal = lineEdit->palette();
        pal.setColor(QPalette::Text, Qt::gray);
        lineEdit->setPalette(pal);
    }
    else if (QLabel *label = qobject_cast<QLabel *>(widget))
    {
        label->setText(currentValues_[key]);
    }
    updateComputedFields();
    updateSendButtonState();
}

bool AppModule::eventFilter(QObject *obj, QEvent *event)
{
    if (QLineEdit *lineEdit = qobject_cast<QLineEdit *>(obj))
    {
        // Identify the key for this QLineEdit.
        QString key;
        for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it)
        {
            if (it.value() == lineEdit)
            {
                key = it.key();
                break;
            }
        }
        if (event->type() == QEvent::FocusIn)
        {
            // On focus in, if the field's text equals the current value, clear it.
            if (lineEdit->text() == currentValues_[key])
                lineEdit->clear();
            QPalette pal = lineEdit->palette();
            pal.setColor(QPalette::Text, Qt::white);
            lineEdit->setPalette(pal);
        }
        else if (event->type() == QEvent::FocusOut)
        {
            // On focus out, if the field is empty, restore the current value.
            if (lineEdit->text().isEmpty())
            {
                lineEdit->setText(currentValues_[key]);
                QPalette pal = lineEdit->palette();
                pal.setColor(QPalette::Text, Qt::gray);
                lineEdit->setPalette(pal);
                // Also clear the override.
                userOverrides_[key].clear();
            }
            // Otherwise, leave the user's input intact.
            updateSendButtonState();
        }
    }
    return QWidget::eventFilter(obj, event);
}
