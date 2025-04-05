#include "manymove_hmi/app_module.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDebug>

AppModule::AppModule(QWidget *parent) : QWidget(parent)
{
    layout_ = new QVBoxLayout(this);

    // Define the keys and how they are handled.

    // tube_length: an editable double field.
    KeyConfig tubeLength;
    tubeLength.key = "tube_length";
    tubeLength.value_type = "double";
    tubeLength.editable = true;
    tubeLength.visible = true;  // visible by default.
    // Identity transformation.
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
        bool ok1 = false, ok2 = false;
        double length = values.value("tube_length").toDouble(&ok1);
        double diameter = values.value("tube_diameter").toDouble(&ok2);
        if (!ok1 || !ok2) {
            // Return empty string if the input is not valid.
            return "";
        }
        // Compute x using the formula: (tube_length / 2) + 0.973 + tube_diameter.
        double x = (length / 2.0) + 0.973 + diameter;
        double y = -0.6465;
        double z = 0.8055;
        double roll = 1.57;
        double pitch = 2.05;
        double yaw = 1.57;
        // Format as a JSON-like string.
        QString poseJson = QString("{\"x\":%1,\"y\":%2,\"z\":%3,\"roll\":%4,\"pitch\":%5,\"yaw\":%6}")
                                .arg(x)
                                .arg(y)
                                .arg(z)
                                .arg(roll)
                                .arg(pitch)
                                .arg(yaw);
        return poseJson;
    };

    // Add all key configurations.
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
        // Create a container widget for the row.
        QWidget *rowWidget = new QWidget(this);
        QHBoxLayout *rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);

        // Create and add a label.
        QLabel *label = new QLabel(config.key, rowWidget);
        rowLayout->addWidget(label);

        if (config.editable) {
            // Create an input field.
            QLineEdit *lineEdit = new QLineEdit(rowWidget);
            // Initialize stored value.
            editableValues_[config.key] = "";
            // Update stored value and computed fields on change.
            connect(lineEdit, &QLineEdit::textChanged, this, &AppModule::onEditableFieldChanged);
            rowLayout->addWidget(lineEdit);
            // Store the field widget if needed later.
            keyWidgets_[config.key] = lineEdit;
        } else {
            // For computed keys, show a non-editable label.
            QLabel *valueLabel = new QLabel("N/A", rowWidget);
            rowLayout->addWidget(valueLabel);
            keyWidgets_[config.key] = valueLabel;
        }
        // Add the container row to the main layout.
        layout_->addWidget(rowWidget);
        // Store the row widget.
        keyRowWidgets_[config.key] = rowWidget;
        // Set the row's visibility based on the configuration.
        rowWidget->setVisible(config.visible);
    }

    // Create a Send button.
    sendButton_ = new QPushButton("Send", this);
    layout_->addWidget(sendButton_);
    connect(sendButton_, &QPushButton::clicked, this, &AppModule::onSendButtonClicked);
}

void AppModule::onEditableFieldChanged(const QString &)
{
    // Update all editable values.
    for (const auto &config : keyConfigs_) {
        if (config.editable) {
            QLineEdit *lineEdit = qobject_cast<QLineEdit*>(keyWidgets_.value(config.key));
            if (lineEdit) {
                editableValues_[config.key] = lineEdit->text();
            }
        }
    }
    // Recalculate computed fields.
    updateComputedFields();
}

void AppModule::updateComputedFields()
{
    // For keys that are computed, update their label with the computed value.
    for (const auto &config : keyConfigs_) {
        if (!config.editable) {
            QString computedValue = config.computeFunction(editableValues_);
            QLabel *label = qobject_cast<QLabel*>(keyWidgets_.value(config.key));
            if (label) {
                label->setText(computedValue);
            }
        }
    }
}

void AppModule::onSendButtonClicked()
{
    // For each key, compute its final value.
    for (const auto &config : keyConfigs_) {
        QString finalValue;
        if (config.editable) {
            // Apply the transformation if provided.
            finalValue = config.computeFunction(editableValues_);
        } else {
            // For computed keys, get the text from the label.
            QLabel *label = qobject_cast<QLabel*>(keyWidgets_.value(config.key));
            if (label) {
                finalValue = label->text();
            }
        }
        if (!finalValue.isEmpty()) {
            emit keyUpdateRequested(config.key, config.value_type, finalValue);
            qDebug() << "Sending key:" << config.key
                     << "Type:" << config.value_type
                     << "Value:" << finalValue;
        } else {
            qDebug() << "Skipping key:" << config.key << "due to empty value.";
        }
    }
}

void AppModule::setKeyVisibility(const QString &key, bool visible)
{
    if (keyRowWidgets_.contains(key)) {
        keyRowWidgets_[key]->setVisible(visible);
    }
}
