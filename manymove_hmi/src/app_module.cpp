#include "manymove_hmi/app_module.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QToolButton>
#include <QPalette>
#include <QEvent>
#include <QDebug>

/* ------------------------------------------------------------------ */
/*  Constructor – build internal state and GUI                        */
/* ------------------------------------------------------------------ */
AppModule::AppModule(const std::vector<KeyConfig> &key_cfg,
                     QWidget *parent)
    : QWidget(parent),
      keyConfigs_(key_cfg)
{
    /* 1) build black-board key list once --------------------------- */
    for (const auto &k : keyConfigs_)
        bb_keys_.push_back({k.key, k.value_type});

    /* 2) initialise state maps ------------------------------------- */
    for (const auto &c : keyConfigs_)
        currentValues_[c.key] = "N/A";

    /* 3) build the visual layout ----------------------------------- */
    setupUI();
}

/* ------------------------------------------------------------------ */
/*  Private helpers                                                   */
/* ------------------------------------------------------------------ */
void AppModule::setupUI()
{
    layout_ = new QVBoxLayout(this);

    for (const auto &cfg : keyConfigs_)
    {
        QWidget *rowWidget = new QWidget(this);
        QHBoxLayout *row = new QHBoxLayout(rowWidget);
        row->setContentsMargins(0, 0, 0, 0);

        QLabel *label = new QLabel(cfg.key, rowWidget);
        label->setFixedWidth(250);
        row->addWidget(label);

        /* ---------------------------------------------------------- */
        if (cfg.value_type == "bool") /* TOGGLE  */
        {
            QToolButton *t = new QToolButton(rowWidget);
            t->setCheckable(true);
            t->setFixedSize(750, 70);
            t->setText("ROBOT CYCLE OFF");
            t->setStyleSheet(
                "QToolButton { background-color: darkred; }"
                "QToolButton:checked { background-color: green; }");

            connect(t, &QToolButton::toggled, this,
                    [this, cfg, t](bool on)
                    {
                        t->setText(on ? "ROBOT CYCLE ON" : "ROBOT CYCLE OFF");
                        const QString v = on ? "true" : "false";
                        userOverrides_[cfg.key] = v;
                        editableValues_[cfg.key] = v;
                        updateComputedFields();
                        updateSendButtonState();
                        if (cfg.key == "cycle_on_key")
                            emit keyUpdateRequested(cfg.key, cfg.value_type, v);
                    });

            row->addWidget(t);
            keyWidgets_[cfg.key] = t;
        }
        else if (cfg.editable) /* LINE EDIT */
        {
            QLineEdit *le = new QLineEdit("N/A", rowWidget);
            le->setAlignment(Qt::AlignRight);
            le->setFixedWidth(750);
            QPalette pal = le->palette();
            pal.setColor(QPalette::Text, Qt::gray);
            le->setPalette(pal);

            connect(le, &QLineEdit::textChanged,
                    this, &AppModule::onEditableChanged);

            le->installEventFilter(this);
            row->addWidget(le);
            keyWidgets_[cfg.key] = le;
        }
        else /* COMPUTED  */
        {
            QLabel *v = new QLabel("N/A", rowWidget);
            v->setAlignment(Qt::AlignRight);
            v->setFixedWidth(750);
            row->addWidget(v);
            keyWidgets_[cfg.key] = v;
        }

        layout_->addWidget(rowWidget);
        keyRowWidgets_[cfg.key] = rowWidget;
        rowWidget->setVisible(cfg.visible);
    }

    sendButton_ = new QPushButton("Send", this);
    layout_->addWidget(sendButton_);
    connect(sendButton_, &QPushButton::clicked,
            this, &AppModule::onSendClicked);

    updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::onEditableChanged(const QString &)
{
    auto *le = qobject_cast<QLineEdit *>(sender());
    if (!le)
        return;

    /* which key? */
    QString key;
    for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it)
        if (it.value() == le)
        {
            key = it.key();
            break;
        }

    if (key.isEmpty())
        return;

    const QString txt = le->text();
    if (txt == currentValues_[key])
    {
        userOverrides_[key].clear();
        editableValues_[key].clear();
    }
    else
    {
        userOverrides_[key] = txt;
        editableValues_[key] = txt;
    }

    /* colouring ---------------------------------------------------- */
    QPalette pal = le->palette();
    bool ok = isFieldValid(key, txt);
    pal.setColor(QPalette::Text,
                 (!txt.isEmpty() && txt != "N/A" && !ok) ? Qt::red : Qt::white);
    le->setPalette(pal);

    updateComputedFields();
    updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::updateComputedFields()
{
    /* Build a map that already contains fall-backs to current values */
    QMap<QString, QString> merged = editableValues_;
    for (const auto &c : keyConfigs_)
        if (merged.value(c.key).isEmpty())
            merged[c.key] = currentValues_[c.key];

    for (const auto &cfg : keyConfigs_)
    {
        if (cfg.editable)
            continue;
        QLabel *lbl = qobject_cast<QLabel *>(keyWidgets_.value(cfg.key));
        if (!lbl)
            continue;

        QString val = cfg.computeFunction ? cfg.computeFunction(merged) : "";
        QPalette pal = lbl->palette();
        if (!val.isEmpty())
        {
            currentValues_[cfg.key] = val;
            lbl->setText(val);
            pal.setColor(QPalette::WindowText, Qt::white);
        }
        else
        {
            lbl->setText(currentValues_.value(cfg.key, "N/A"));
            pal.setColor(QPalette::WindowText, Qt::gray);
        }
        lbl->setPalette(pal);
    }
}

/* ------------------------------------------------------------------ */
void AppModule::updateSendButtonState()
{
    /* 1) ‘cycle_on_key’ must be off -------------------------------- */
    if (auto *btn = qobject_cast<QAbstractButton *>(keyWidgets_.value("cycle_on_key")))
        if (btn->isChecked())
        {
            sendButton_->setEnabled(false);
            return;
        }

    /* 2) at least one valid edit & no invalid ones ----------------- */
    bool good = false, bad = false;
    for (const auto &cfg : keyConfigs_)
    {
        if (!cfg.editable)
            continue;
        const QString txt = userOverrides_.value(cfg.key);
        if (txt.isEmpty() || txt == currentValues_[cfg.key])
            continue;
        if (isFieldValid(cfg.key, txt))
            good = true;
        else
        {
            bad = true;
            break;
        }
    }
    sendButton_->setEnabled(good && !bad);
}

/* ------------------------------------------------------------------ */
bool AppModule::isFieldValid(const QString &key, const QString &txt) const
{
    if (txt.isEmpty() || txt == "N/A" || txt == currentValues_.value(key))
        return false;

    for (const auto &c : keyConfigs_)
        if (c.key == key && c.value_type == "double")
        {
            bool ok = false;
            txt.toDouble(&ok);
            return ok;
        }
    return true;
}

/* ------------------------------------------------------------------ */
void AppModule::onSendClicked()
{
    for (const auto &cfg : keyConfigs_)
    {
        QString final;
        if (cfg.editable)
        {
            QString raw = userOverrides_.value(cfg.key);
            if (raw.isEmpty() || raw == currentValues_[cfg.key] || !isFieldValid(cfg.key, raw))
                continue;
            final = cfg.computeFunction ? cfg.computeFunction(editableValues_) : raw;
        }
        else
        {
            auto *lbl = qobject_cast<QLabel *>(keyWidgets_.value(cfg.key));
            if (!lbl)
                continue;
            final = lbl->text();
            if (final == "N/A" || final.isEmpty())
                continue;
        }
        emit keyUpdateRequested(cfg.key, cfg.value_type, final);
    }

    /* clear overrides --------------------------------------------- */
    for (const auto &cfg : keyConfigs_)
        if (cfg.editable)
        {
            userOverrides_[cfg.key].clear();
            if (auto *le = qobject_cast<QLineEdit *>(keyWidgets_[cfg.key]))
            {
                le->setText(currentValues_[cfg.key]);
                QPalette pal = le->palette();
                pal.setColor(QPalette::Text, Qt::gray);
                le->setPalette(pal);
            }
        }
    updateComputedFields();
    updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::setKeyVisibility(const QString &key, bool v)
{
    if (keyRowWidgets_.contains(key))
        keyRowWidgets_[key]->setVisible(v);
}

/* ------------------------------------------------------------------ */
void AppModule::updateField(const QString &key, const QString &newVal)
{
    if (!keyWidgets_.contains(key))
        return;
    currentValues_[key] = newVal.isEmpty() ? "N/A" : newVal;

    if (auto *le = qobject_cast<QLineEdit *>(keyWidgets_[key]))
    {
        if (le->hasFocus() || (!userOverrides_[key].isEmpty() && userOverrides_[key] != currentValues_[key]))
            return;
        le->setText(currentValues_[key]);
        QPalette pal = le->palette();
        pal.setColor(QPalette::Text, Qt::gray);
        le->setPalette(pal);
    }
    else if (auto *lbl = qobject_cast<QLabel *>(keyWidgets_[key]))
        lbl->setText(currentValues_[key]);

    updateComputedFields();
    updateSendButtonState();
}

/* ------------------------------------------------------------------ */
bool AppModule::eventFilter(QObject *o, QEvent *e)
{
    if (auto *le = qobject_cast<QLineEdit *>(o))
    {
        QString key;
        for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it)
            if (it.value() == le)
            {
                key = it.key();
                break;
            }

        if (e->type() == QEvent::FocusIn)
        {
            if (le->text() == currentValues_[key])
                le->clear();
            QPalette p = le->palette();
            p.setColor(QPalette::Text, Qt::white);
            le->setPalette(p);
        }
        else if (e->type() == QEvent::FocusOut)
        {
            if (le->text().isEmpty())
            {
                le->setText(currentValues_[key]);
                QPalette p = le->palette();
                p.setColor(QPalette::Text, Qt::gray);
                le->setPalette(p);
                userOverrides_[key].clear();
            }
            updateSendButtonState();
        }
    }
    return QWidget::eventFilter(o, e);
}
