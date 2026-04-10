#pragma once

#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>

#include <lccv.hpp>

// ---------------------------------------------------------------------------
// SettingsDialog
//
// Exposes EV compensation, white balance, and metering mode.
// Changes are applied to the Options object when the user clicks OK.
// The caller is responsible for restarting the camera to make them effective.
// ---------------------------------------------------------------------------

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    // opts must remain valid for the lifetime of the dialog.
    explicit SettingsDialog(lccv::Options *opts, QWidget *parent = nullptr);

private Q_SLOTS:
    void onEvSliderMoved(int value);
    void accept() override;

private:
    void buildUi();
    void applyStyleSheet();
    void loadFromOptions();
    void saveToOptions();

    lccv::Options *opts_;

    QComboBox    *wbCombo_;
    QComboBox    *meteringCombo_;
    QComboBox    *exposureCombo_;
    QSlider      *evSlider_;    // -4 .. +4, maps to -2.0 .. +2.0 EV in steps of 0.5
    QLabel       *evValueLabel_;
    QDoubleSpinBox *brightnessSpin_;
};
