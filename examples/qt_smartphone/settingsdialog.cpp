#include "settingsdialog.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QGroupBox>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

SettingsDialog::SettingsDialog(lccv::Options *opts, QWidget *parent)
    : QDialog(parent), opts_(opts)
{
    setWindowTitle("Camera Settings");
    setModal(true);
    setMinimumWidth(340);
    buildUi();
    applyStyleSheet();
    loadFromOptions();
}

void SettingsDialog::buildUi()
{
    // ── White balance ────────────────────────────────────────────────────────
    wbCombo_ = new QComboBox(this);
    wbCombo_->addItem("Auto",         QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::AUTO)));
    wbCombo_->addItem("Incandescent", QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::INCANDESCENT)));
    wbCombo_->addItem("Tungsten",     QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::TUNGSTEN)));
    wbCombo_->addItem("Fluorescent",  QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::FLUORESCENT)));
    wbCombo_->addItem("Indoor",       QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::INDOOR)));
    wbCombo_->addItem("Daylight",     QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::DAYLIGHT)));
    wbCombo_->addItem("Cloudy",       QVariant::fromValue(
                          static_cast<int>(lccv::WhiteBalance::CLOUDY)));

    // ── Metering ─────────────────────────────────────────────────────────────
    meteringCombo_ = new QComboBox(this);
    meteringCombo_->addItem("Matrix",  QVariant::fromValue(
                                static_cast<int>(lccv::Metering::MATRIX)));
    meteringCombo_->addItem("Centre",  QVariant::fromValue(
                                static_cast<int>(lccv::Metering::CENTRE)));
    meteringCombo_->addItem("Spot",    QVariant::fromValue(
                                static_cast<int>(lccv::Metering::SPOT)));

    // ── Exposure mode ────────────────────────────────────────────────────────
    exposureCombo_ = new QComboBox(this);
    exposureCombo_->addItem("Normal", QVariant::fromValue(
                                static_cast<int>(lccv::Exposure::NORMAL)));
    exposureCombo_->addItem("Short",  QVariant::fromValue(
                                static_cast<int>(lccv::Exposure::SHORT)));

    // ── EV slider  (-2.0 … +2.0 in 0.5 steps → slider 0..8, centre=4) ──────
    evSlider_ = new QSlider(Qt::Horizontal, this);
    evSlider_->setRange(-4, 4);   // actual EV = value * 0.5
    evSlider_->setTickPosition(QSlider::TicksBelow);
    evSlider_->setTickInterval(1);
    evSlider_->setSingleStep(1);

    evValueLabel_ = new QLabel("0.0", this);
    evValueLabel_->setMinimumWidth(36);
    evValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QHBoxLayout *evRow = new QHBoxLayout;
    evRow->addWidget(new QLabel("-2", this));
    evRow->addWidget(evSlider_, 1);
    evRow->addWidget(new QLabel("+2", this));
    evRow->addWidget(evValueLabel_);

    connect(evSlider_, &QSlider::valueChanged,
            this, &SettingsDialog::onEvSliderMoved);

    // ── Brightness ───────────────────────────────────────────────────────────
    brightnessSpin_ = new QDoubleSpinBox(this);
    brightnessSpin_->setRange(-1.0, 1.0);
    brightnessSpin_->setSingleStep(0.05);
    brightnessSpin_->setDecimals(2);

    // ── Form layout ──────────────────────────────────────────────────────────
    QGroupBox *group = new QGroupBox("Image controls", this);
    QFormLayout *form = new QFormLayout(group);
    form->setLabelAlignment(Qt::AlignRight);
    form->addRow("White balance:",  wbCombo_);
    form->addRow("Metering:",       meteringCombo_);
    form->addRow("Exposure mode:",  exposureCombo_);
    form->addRow("EV compensation:", evRow);
    form->addRow("Brightness:",     brightnessSpin_);

    // ── Buttons ──────────────────────────────────────────────────────────────
    QDialogButtonBox *buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &SettingsDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    // ── Root layout ──────────────────────────────────────────────────────────
    QVBoxLayout *root = new QVBoxLayout(this);
    root->addWidget(group);
    root->addWidget(buttons);
}

void SettingsDialog::applyStyleSheet()
{
    setStyleSheet(R"(
        QDialog, QWidget {
            background-color: #1e1e1e;
            color: #f0f0f0;
        }
        QGroupBox {
            border: 1px solid #444444;
            border-radius: 6px;
            margin-top: 8px;
            padding-top: 8px;
            font-weight: bold;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            color: #aaaaaa;
        }
        QComboBox, QDoubleSpinBox {
            background-color: #2a2a2a;
            color: #f0f0f0;
            border: 1px solid #555555;
            border-radius: 4px;
            padding: 4px;
            min-height: 26px;
        }
        QComboBox::drop-down { border: none; }
        QComboBox QAbstractItemView {
            background-color: #2a2a2a;
            selection-background-color: #0078d4;
        }
        QSlider::groove:horizontal {
            height: 4px;
            background: #444444;
            border-radius: 2px;
        }
        QSlider::handle:horizontal {
            background: #0078d4;
            width: 16px;
            height: 16px;
            margin: -6px 0;
            border-radius: 8px;
        }
        QPushButton {
            background-color: #2a2a2a;
            color: #f0f0f0;
            border: 1px solid #555555;
            border-radius: 4px;
            padding: 6px 16px;
            min-width: 70px;
        }
        QPushButton:default {
            background-color: #0078d4;
            border: 1px solid #005ea2;
        }
        QPushButton:pressed { background-color: #444444; }
    )");
}

// ---------------------------------------------------------------------------
// Load / save
// ---------------------------------------------------------------------------

void SettingsDialog::loadFromOptions()
{
    // White balance
    int wbVal = opts_->getWhiteBalance();
    for (int i = 0; i < wbCombo_->count(); ++i) {
        if (wbCombo_->itemData(i).toInt() == wbVal) {
            wbCombo_->setCurrentIndex(i);
            break;
        }
    }

    // Metering
    int mVal = opts_->getMeteringMode();
    for (int i = 0; i < meteringCombo_->count(); ++i) {
        if (meteringCombo_->itemData(i).toInt() == mVal) {
            meteringCombo_->setCurrentIndex(i);
            break;
        }
    }

    // Exposure mode
    int eVal = opts_->getExposureMode();
    for (int i = 0; i < exposureCombo_->count(); ++i) {
        if (exposureCombo_->itemData(i).toInt() == eVal) {
            exposureCombo_->setCurrentIndex(i);
            break;
        }
    }

    // EV — slider range is -4..+4, where value*0.5 == EV
    float ev = opts_->ev;
    ev = std::max(-2.0f, std::min(2.0f, ev));
    evSlider_->setValue(static_cast<int>(std::round(ev / 0.5f)));
    evValueLabel_->setText(QString("%1").arg(static_cast<double>(ev), 0, 'f', 1));

    // Brightness
    brightnessSpin_->setValue(static_cast<double>(opts_->brightness));
}

void SettingsDialog::saveToOptions()
{
    opts_->setWhiteBalance(
        static_cast<lccv::WhiteBalance>(wbCombo_->currentData().toInt()));
    opts_->setMetering(
        static_cast<lccv::Metering>(meteringCombo_->currentData().toInt()));
    opts_->setExposureMode(
        static_cast<lccv::Exposure>(exposureCombo_->currentData().toInt()));
    opts_->ev         = static_cast<float>(evSlider_->value()) * 0.5f;
    opts_->brightness = static_cast<float>(brightnessSpin_->value());
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void SettingsDialog::onEvSliderMoved(int value)
{
    float ev = static_cast<float>(value) * 0.5f;
    evValueLabel_->setText(QString("%1").arg(static_cast<double>(ev), 0, 'f', 1));
}

void SettingsDialog::accept()
{
    saveToOptions();
    QDialog::accept();
}
