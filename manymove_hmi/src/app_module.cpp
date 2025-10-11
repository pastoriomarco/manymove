// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "manymove_hmi/app_module.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QEvent>
#include <QDebug>
#include <QFont>
#include <QSizePolicy>

namespace
{
  inline double scaleFor(const std::vector<KeyConfig>& cfgs, const QString& key)
  {
    for (const auto& c : cfgs) {
      if (c.key == key) {
        return c.display_scale;
      }
    }
    return 1.0;
  }

  inline QString convertFromInternal(
    const std::vector<KeyConfig>& cfgs,
    const QString& key, const QString& val)
  {
    double scale = scaleFor(cfgs, key);
    if (scale == 1.0) {
      return val;
    }
    bool ok = false;
    double v = val.toDouble(&ok);
    return ok ? QString::number(v * scale) : val;
  }

  inline QString convertToInternal(
    const std::vector<KeyConfig>& cfgs,
    const QString& key, const QString& val)
  {
    double scale = scaleFor(cfgs, key);
    if (scale == 1.0) {
      return val;
    }
    bool ok = false;
    double v = val.toDouble(&ok);
    return ok ? QString::number(v / scale) : val;
  }
} // namespace

/* ------------------------------------------------------------------ */
/*  Constructor – build internal state and GUI                        */
/* ------------------------------------------------------------------ */
AppModule::AppModule(
  const std::vector<KeyConfig>& key_cfg,
  QWidget* parent)
  : QWidget(parent),
  keyConfigs_(key_cfg)
{
  /* 1) build black-board key list once --------------------------- */
  for (const auto& k : keyConfigs_) {
    bb_keys_.push_back({k.key, k.value_type});
  }

  /* 2) initialise state maps ------------------------------------- */
  for (const auto& c : keyConfigs_) {
    currentValues_[c.key] = "N/A";
  }

  /* 3) build the visual layout ----------------------------------- */
  setupUI();
}

/* ------------------------------------------------------------------ */
/*  Private helpers                                                   */
/* ------------------------------------------------------------------ */
void AppModule::setupUI()
{
  layout_ = new QVBoxLayout(this);

  for (const auto& cfg : keyConfigs_) {
    QWidget* rowWidget = new QWidget(this);
    QHBoxLayout* row = new QHBoxLayout(rowWidget);
    row->setContentsMargins(0, 0, 0, 0);

    if (cfg.show_label) {
      QLabel* label = new QLabel(cfg.key, rowWidget);
      label->setFixedWidth(250);
      QFont labelFont = label->font();
      labelFont.setPointSize(14);
      label->setFont(labelFont);
      row->addWidget(label);
    }

    if (!cfg.unit.isEmpty()) {
      QLabel* unitLbl = new QLabel(cfg.unit, rowWidget);
      unitLbl->setFixedWidth(60);
      row->addWidget(unitLbl);
    }

    /* ---------------------------------------------------------- */
    if (cfg.value_type == "bool") {   /* TOGGLE  */
      QToolButton* t = new QToolButton(rowWidget);
      t->setCheckable(true);
      if (cfg.widget_width > 0) {
        t->setFixedWidth(cfg.widget_width);
      }
      else {
        t->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      }
      t->setFixedHeight(70);
      t->setText("ROBOT CYCLE OFF");
      t->setStyleSheet("QToolButton { background-color: darkred; }"
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
        if (cfg.key == "cycle_on_key") {
          emit keyUpdateRequested(cfg.key, cfg.value_type, v);
        }
      });

      row->addWidget(t);
      keyWidgets_[cfg.key] = t;
    }
    else if (cfg.editable) {   /* LINE EDIT */
      QLineEdit* le = new QLineEdit("N/A", rowWidget);
      le->setAlignment(Qt::AlignRight);
      le->setFixedHeight(50);
      QFont font = le->font();
      font.setPointSize(20);
      le->setFont(font);
      le->setFixedWidth(cfg.widget_width);
      QPalette pal = le->palette();
      pal.setColor(QPalette::Text, Qt::gray);
      le->setPalette(pal);

      connect(le, &QLineEdit::textChanged,
              this, &AppModule::onEditableChanged);

      le->installEventFilter(this);
      row->addWidget(le);
      keyWidgets_[cfg.key] = le;
    }
    else {   /* COMPUTED  */
      QLabel* v = new QLabel("N/A", rowWidget);
      v->setAlignment(Qt::AlignRight);
      v->setFixedWidth(cfg.widget_width);
      row->addWidget(v);
      keyWidgets_[cfg.key] = v;
    }

    layout_->addWidget(rowWidget);
    keyRowWidgets_[cfg.key] = rowWidget;
    rowWidget->setVisible(cfg.visible);
  }

  sendButton_ = new QPushButton("SEND", this);
  layout_->addWidget(sendButton_);
  connect(sendButton_, &QPushButton::clicked,
          this, &AppModule::onSendClicked);

  generalMessage_ = new QLabel(this);
  generalMessage_->setFixedHeight(50);
  QFont genFont = generalMessage_->font();
  genFont.setPointSize(14);
  generalMessage_->setFont(genFont);
  layout_->addWidget(generalMessage_);

  updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::updateGeneralMessage(const QString& message, const QString& color)
{
  if (!generalMessage_) {
    return;
  }
  generalMessage_->setText(message);
  if (!color.isEmpty()) {
    generalMessage_->setStyleSheet(QString("color: %1; border: 2px solid %1; padding:2px;").arg(
      color));
  }
  else {
    generalMessage_->setStyleSheet("");
  }
}

/* ------------------------------------------------------------------ */
void AppModule::onEditableChanged(const QString&)
{
  auto* le = qobject_cast<QLineEdit*>(sender());
  if (!le) {
    return;
  }

  /* which key? */
  QString key;
  for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it) {
    if (it.value() == le) {
      key = it.key();
      break;
    }
  }

  if (key.isEmpty()) {
    return;
  }

  const QString txt = le->text();
  QString in_m = toInternal(key, txt);
  if (in_m == currentValues_[key]) {
    userOverrides_[key].clear();
    editableValues_[key].clear();
  }
  else {
    userOverrides_[key] = in_m;
    editableValues_[key] = in_m;
  }

  /* colouring ---------------------------------------------------- */
  QPalette pal = le->palette();
  FieldStatus st = validateField(key, in_m);
  if (!txt.isEmpty() && txt != "N/A") {
    switch (st) {
      case FieldStatus::Invalid:
        pal.setColor(QPalette::Text, Qt::red);
        break;
      case FieldStatus::BelowLimit:
        pal.setColor(QPalette::Text, Qt::cyan);
        break;
      case FieldStatus::AboveLimit:
        pal.setColor(QPalette::Text, Qt::magenta);
        break;
      case FieldStatus::Valid:
      default:
        pal.setColor(QPalette::Text, Qt::white);
        break;
    }
  }
  else {
    pal.setColor(QPalette::Text, Qt::white);
  }
  le->setPalette(pal);

  updateComputedFields();
  updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::updateComputedFields()
{
  /* Build a map that already contains fall-backs to current values */
  QMap<QString, QString> merged = editableValues_;
  for (const auto& c : keyConfigs_) {
    if (merged.value(c.key).isEmpty()) {
      merged[c.key] = currentValues_[c.key];
    }
  }

  for (const auto& cfg : keyConfigs_) {
    if (cfg.editable) {
      continue;
    }
    QLabel* lbl = qobject_cast<QLabel*>(keyWidgets_.value(cfg.key));
    if (!lbl) {
      continue;
    }

    QString val = cfg.computeFunction ? cfg.computeFunction(merged) : "";
    QPalette pal = lbl->palette();
    if (!val.isEmpty()) {
      currentValues_[cfg.key] = val;
      lbl->setText(toDisplay(cfg.key, val));
      pal.setColor(QPalette::WindowText, Qt::white);
    }
    else {
      lbl->setText(toDisplay(cfg.key,
                             currentValues_.value(cfg.key, "N/A")));
      pal.setColor(QPalette::WindowText, Qt::gray);
    }
    lbl->setPalette(pal);
  }
}

/* ------------------------------------------------------------------ */
void AppModule::updateSendButtonState()
{
  /* 1) ‘cycle_on_key’ must be off -------------------------------- */
  if (auto* btn = qobject_cast<QAbstractButton*>(keyWidgets_.value("cycle_on_key"))) {
    if (btn->isChecked()) {
      sendButton_->setEnabled(false);
      return;
    }
  }

  /* 2) at least one valid edit & no invalid ones ----------------- */
  bool good = false, bad = false;
  for (const auto& cfg : keyConfigs_) {
    if (!cfg.editable) {
      continue;
    }
    const QString txt = userOverrides_.value(cfg.key);
    if (txt.isEmpty() || txt == currentValues_[cfg.key]) {
      continue;
    }
    if (isFieldValid(cfg.key, txt)) {
      good = true;
    }
    else {
      bad = true;
      break;
    }
  }
  sendButton_->setEnabled(good && !bad);
}

/* ------------------------------------------------------------------ */
FieldStatus AppModule::validateField(const QString& key, const QString& txt) const
{
  if (txt.isEmpty() || txt == "N/A" || txt == currentValues_.value(key)) {
    return FieldStatus::Invalid;
  }

  for (const auto& c : keyConfigs_) {
    if (c.key == key && c.value_type == "double") {
      bool ok = false;
      double v = txt.toDouble(&ok);
      if (!ok) {
        return FieldStatus::Invalid;
      }

      if (!std::isnan(c.lower_limit) && v < c.lower_limit) {
        return FieldStatus::BelowLimit;
      }
      if (!std::isnan(c.upper_limit) && v > c.upper_limit) {
        return FieldStatus::AboveLimit;
      }
      return FieldStatus::Valid;
    }
  }

  return txt.isEmpty() ? FieldStatus::Invalid : FieldStatus::Valid;
}

/* ------------------------------------------------------------------ */
bool AppModule::isFieldValid(const QString& key, const QString& txt) const
{
  return validateField(key, txt) == FieldStatus::Valid;
}

/* ------------------------------------------------------------------ */
void AppModule::onSendClicked()
{
  for (const auto& cfg : keyConfigs_) {
    QString final;
    if (cfg.editable) {
      QString raw = userOverrides_.value(cfg.key);
      if (raw.isEmpty() || raw == currentValues_[cfg.key] || !isFieldValid(cfg.key, raw)) {
        continue;
      }
      final = cfg.computeFunction ? cfg.computeFunction(editableValues_) : raw;
    }
    else {
      auto* lbl = qobject_cast<QLabel*>(keyWidgets_.value(cfg.key));
      if (!lbl) {
        continue;
      }
      final = lbl->text();
      if (final == "N/A" || final.isEmpty()) {
        continue;
      }
    }
    emit keyUpdateRequested(cfg.key, cfg.value_type, final);
    currentValues_[cfg.key] = final;
  }

  /* clear overrides --------------------------------------------- */
  for (const auto& cfg : keyConfigs_) {
    if (cfg.editable) {
      userOverrides_[cfg.key].clear();
      if (auto* le = qobject_cast<QLineEdit*>(keyWidgets_[cfg.key])) {
        QString displayVal = toDisplay(cfg.key, currentValues_[cfg.key]);
        le->setText(displayVal);
        QPalette pal = le->palette();
        pal.setColor(QPalette::Text, Qt::gray);
        le->setPalette(pal);
      }
    }
  }
  updateComputedFields();
  updateSendButtonState();
}

/* ------------------------------------------------------------------ */
void AppModule::setKeyVisibility(const QString& key, bool v)
{
  if (keyRowWidgets_.contains(key)) {
    keyRowWidgets_[key]->setVisible(v);
  }
}

/* ------------------------------------------------------------------ */
void AppModule::updateField(const QString& key, const QString& newVal)
{
  if (!keyWidgets_.contains(key)) {
    return;
  }

  // Normalize numeric values so that comparisons with user edits are stable
  QString normalized = newVal;
  if (!normalized.isEmpty()) {
    for (const auto& cfg : keyConfigs_) {
      if (cfg.key == key && cfg.value_type == "double") {
        bool ok = false;
        double d = newVal.toDouble(&ok);
        if (ok) {
          normalized = QString::number(d);
        }
        break;
      }
    }
  }

  currentValues_[key] = normalized.isEmpty() ? "N/A" : normalized;

  if (auto* le = qobject_cast<QLineEdit*>(keyWidgets_[key])) {
    if (le->hasFocus() ||
        (!userOverrides_[key].isEmpty() && userOverrides_[key] != currentValues_[key])) {
      return;
    }
    QString displayVal = toDisplay(key, currentValues_[key]);

    le->blockSignals(true);
    le->setText(displayVal);
    le->blockSignals(false);

    QPalette pal = le->palette();
    pal.setColor(QPalette::Text, Qt::gray);
    le->setPalette(pal);
  }
  else if (auto* lbl = qobject_cast<QLabel*>(keyWidgets_[key])) {
    QString displayVal = toDisplay(key, currentValues_[key]);
    lbl->setText(displayVal);
  }

  updateComputedFields();
  updateSendButtonState();
}

/* ------------------------------------------------------------------ */
bool AppModule::eventFilter(QObject* o, QEvent* e)
{
  if (auto* le = qobject_cast<QLineEdit*>(o)) {
    QString key;
    for (auto it = keyWidgets_.cbegin(); it != keyWidgets_.cend(); ++it) {
      if (it.value() == le) {
        key = it.key();
        break;
      }
    }

    if (e->type() == QEvent::FocusIn) {
      QString displayVal = toDisplay(key, currentValues_[key]);
      if (le->text() == displayVal) {
        le->clear();
      }
      QPalette p = le->palette();
      p.setColor(QPalette::Text, Qt::white);
      le->setPalette(p);
    }
    else if (e->type() == QEvent::FocusOut) {
      if (le->text().isEmpty()) {
        QString displayVal = toDisplay(key, currentValues_[key]);
        le->setText(displayVal);
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

/* ------------------------------------------------------------------ */
QString AppModule::toDisplay(const QString& key, const QString& val) const
{
  return convertFromInternal(keyConfigs_, key, val);
}

/* ------------------------------------------------------------------ */
QString AppModule::toInternal(const QString& key, const QString& val) const
{
  return convertToInternal(keyConfigs_, key, val);
}
