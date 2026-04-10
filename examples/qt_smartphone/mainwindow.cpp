#include "mainwindow.h"
#include "settingsdialog.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QDateTime>
#include <QSizePolicy>
#include <QStatusBar>

#include <iostream>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    buildUi();
    applyStyleSheet();

    // Cross-thread frame delivery: dispatcher thread emits, GUI thread receives.
    connect(this, &MainWindow::frameReady, this, &MainWindow::updateFrame,
            Qt::QueuedConnection);

    // Recording timer — ticks every second to update the elapsed display.
    recTimer_ = new QTimer(this);
    recTimer_->setInterval(1000);
    connect(recTimer_, &QTimer::timeout, this, &MainWindow::tickRecordingTimer);

    setMinimumSize(820, 500);
    resize(920, 540);

    // Start camera in photo mode.
    setMode(Mode::Photo);
    startCameraPhotoMode();
}

MainWindow::~MainWindow()
{
    stopCamera();
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void MainWindow::buildUi()
{
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    // ── Viewfinder ──────────────────────────────────────────────────────────
    viewfinderLabel_ = new QLabel(this);
    viewfinderLabel_->setAlignment(Qt::AlignCenter);
    viewfinderLabel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    viewfinderLabel_->setMinimumSize(480, 360);
    viewfinderLabel_->setText("Starting camera…");

    // ── Mode buttons ────────────────────────────────────────────────────────
    photoModeBtn_ = new QPushButton("Photo", this);
    videoModeBtn_ = new QPushButton("Video", this);
    photoModeBtn_->setCheckable(true);
    videoModeBtn_->setCheckable(true);
    photoModeBtn_->setChecked(true);
    photoModeBtn_->setFixedHeight(36);
    videoModeBtn_->setFixedHeight(36);

    connect(photoModeBtn_, &QPushButton::clicked, this, &MainWindow::onPhotoModeClicked);
    connect(videoModeBtn_, &QPushButton::clicked, this, &MainWindow::onVideoModeClicked);

    QHBoxLayout *modeLay = new QHBoxLayout;
    modeLay->setSpacing(4);
    modeLay->addWidget(photoModeBtn_);
    modeLay->addWidget(videoModeBtn_);

    // ── Recording indicator ─────────────────────────────────────────────────
    recordingDot_ = new QLabel("●", this);
    recordingDot_->setAlignment(Qt::AlignCenter);
    recordingDot_->setVisible(false);

    recordingTime_ = new QLabel("00:00", this);
    recordingTime_->setAlignment(Qt::AlignCenter);
    recordingTime_->setVisible(false);

    QHBoxLayout *recIndLay = new QHBoxLayout;
    recIndLay->addWidget(recordingDot_);
    recIndLay->addWidget(recordingTime_);

    // ── Shutter button ──────────────────────────────────────────────────────
    shutterButton_ = new QPushButton(this);
    shutterButton_->setFixedSize(72, 72);
    shutterButton_->setToolTip("Capture / Record");
    connect(shutterButton_, &QPushButton::clicked, this, &MainWindow::onShutterClicked);

    // ── Zoom controls ───────────────────────────────────────────────────────
    zoomLabel_ = new QLabel("1.0×", this);
    zoomLabel_->setAlignment(Qt::AlignCenter);

    zoomInButton_  = new QPushButton("+", this);
    zoomOutButton_ = new QPushButton("−", this);
    zoomInButton_->setFixedHeight(40);
    zoomOutButton_->setFixedHeight(40);
    zoomInButton_->setToolTip("Zoom in");
    zoomOutButton_->setToolTip("Zoom out");

    connect(zoomInButton_,  &QPushButton::clicked, this, &MainWindow::onZoomIn);
    connect(zoomOutButton_, &QPushButton::clicked, this, &MainWindow::onZoomOut);

    // ── Settings button ─────────────────────────────────────────────────────
    settingsButton_ = new QPushButton("Settings", this);
    settingsButton_->setFixedHeight(40);
    connect(settingsButton_, &QPushButton::clicked, this, &MainWindow::onSettingsClicked);

    // ── Status bar ──────────────────────────────────────────────────────────
    statusLabel_ = new QLabel("Ready", this);
    statusLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    statusBar()->addWidget(statusLabel_, 1);

    // ── Toolbar layout (right column) ───────────────────────────────────────
    QWidget *toolbar = new QWidget(this);
    toolbar->setFixedWidth(160);
    toolbar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

    QVBoxLayout *tbLay = new QVBoxLayout(toolbar);
    tbLay->setContentsMargins(8, 12, 8, 12);
    tbLay->setSpacing(10);

    tbLay->addLayout(modeLay);
    tbLay->addLayout(recIndLay);
    tbLay->addStretch(1);
    tbLay->addWidget(shutterButton_, 0, Qt::AlignHCenter);
    tbLay->addStretch(1);
    tbLay->addWidget(zoomLabel_);
    tbLay->addWidget(zoomInButton_);
    tbLay->addWidget(zoomOutButton_);
    tbLay->addStretch(2);
    tbLay->addWidget(settingsButton_);

    // ── Root layout ─────────────────────────────────────────────────────────
    QHBoxLayout *rootLay = new QHBoxLayout(central);
    rootLay->setContentsMargins(0, 0, 0, 0);
    rootLay->setSpacing(0);
    rootLay->addWidget(viewfinderLabel_, 1);
    rootLay->addWidget(toolbar);
}

void MainWindow::applyStyleSheet()
{
    // Dark smartphone-style theme.
    setStyleSheet(R"(
        QMainWindow, QWidget {
            background-color: #1a1a1a;
            color: #f0f0f0;
        }
        QStatusBar {
            background-color: #111111;
            color: #888888;
            font-size: 11px;
        }
        QLabel#viewfinderLabel {
            background-color: #000000;
        }
    )");

    viewfinderLabel_->setObjectName("viewfinderLabel");
    viewfinderLabel_->setStyleSheet("background-color: #000000;");

    // Mode buttons
    const QString modeBtnStyle = R"(
        QPushButton {
            background-color: #333333;
            color: #cccccc;
            border: 1px solid #555555;
            border-radius: 4px;
            font-size: 13px;
            padding: 4px;
        }
        QPushButton:checked {
            background-color: #0078d4;
            color: #ffffff;
            border: 1px solid #005ea2;
        }
        QPushButton:hover:!checked {
            background-color: #444444;
        }
    )";
    photoModeBtn_->setStyleSheet(modeBtnStyle);
    videoModeBtn_->setStyleSheet(modeBtnStyle);

    // Shutter button — white circle, turns red when recording
    shutterButton_->setStyleSheet(R"(
        QPushButton {
            background-color: #ffffff;
            border: 4px solid #888888;
            border-radius: 36px;
        }
        QPushButton:pressed {
            background-color: #dddddd;
        }
    )");

    // Recording dot
    recordingDot_->setStyleSheet("color: #ff3030; font-size: 18px;");
    recordingTime_->setStyleSheet("color: #ff3030; font-size: 13px; font-weight: bold;");

    // Zoom label
    zoomLabel_->setStyleSheet("color: #ffcc00; font-size: 14px; font-weight: bold;");

    // Zoom buttons
    const QString zoomBtnStyle = R"(
        QPushButton {
            background-color: #2a2a2a;
            color: #f0f0f0;
            border: 1px solid #555555;
            border-radius: 4px;
            font-size: 20px;
            font-weight: bold;
        }
        QPushButton:pressed { background-color: #444444; }
        QPushButton:disabled { color: #555555; }
    )";
    zoomInButton_->setStyleSheet(zoomBtnStyle);
    zoomOutButton_->setStyleSheet(zoomBtnStyle);

    // Settings button
    settingsButton_->setStyleSheet(R"(
        QPushButton {
            background-color: #2a2a2a;
            color: #cccccc;
            border: 1px solid #555555;
            border-radius: 4px;
            font-size: 13px;
            padding: 4px;
        }
        QPushButton:pressed { background-color: #444444; }
    )");
}

// ---------------------------------------------------------------------------
// Mode management
// ---------------------------------------------------------------------------

void MainWindow::setMode(Mode m)
{
    mode_ = m;
    photoModeBtn_->setChecked(m == Mode::Photo);
    videoModeBtn_->setChecked(m == Mode::Video);

    if (m == Mode::Photo) {
        shutterButton_->setToolTip("Take photo");
        shutterButton_->setStyleSheet(R"(
            QPushButton {
                background-color: #ffffff;
                border: 4px solid #888888;
                border-radius: 36px;
            }
            QPushButton:pressed { background-color: #dddddd; }
        )");
    } else {
        shutterButton_->setToolTip("Start / stop recording");
        shutterButton_->setStyleSheet(R"(
            QPushButton {
                background-color: #ffffff;
                border: 4px solid #cc0000;
                border-radius: 36px;
            }
            QPushButton:pressed { background-color: #dddddd; }
        )");
    }
}

// ---------------------------------------------------------------------------
// Camera lifecycle
// ---------------------------------------------------------------------------

void MainWindow::attachViewfinderCallback()
{
    cam_.startViewfinder([this](cv::Mat &frame) {
        // Running on the LCCV dispatcher thread — must not touch Qt widgets here.
        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        // QImage::copy() ensures the image owns its buffer before rgb goes out of scope.
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step),
                   QImage::Format_RGB888);
        Q_EMIT frameReady(img.copy());
    });
}

void MainWindow::startCameraPhotoMode()
{
    cam_.options->photo_width       = 1280;
    cam_.options->photo_height      = 720;
    cam_.options->viewfinder_width  = 640;
    cam_.options->viewfinder_height = 480;
    cam_.options->framerate         = 30;

    if (!cam_.startPhoto()) {
        QMessageBox::critical(this, "Camera error",
                              "Failed to start camera in photo mode.");
        return;
    }
    attachViewfinderCallback();
    camRunning_ = true;
    setStatus("Photo mode — press shutter to capture");
}

void MainWindow::startCameraVideoMode()
{
    cam_.options->video_width       = 1280;
    cam_.options->video_height      = 720;
    cam_.options->viewfinder_width  = 640;
    cam_.options->viewfinder_height = 480;
    cam_.options->framerate         = 30;

    if (!cam_.startVideo()) {
        QMessageBox::critical(this, "Camera error",
                              "Failed to start camera in video mode.");
        return;
    }
    attachViewfinderCallback();
    camRunning_ = true;
    setStatus("Video mode — press shutter to record");
}

void MainWindow::stopCamera()
{
    if (!camRunning_) return;

    if (recordingActive_.load())
        stopRecording();

    cam_.stopViewfinder();

    if (mode_ == Mode::Photo)
        cam_.stopPhoto();
    else
        cam_.stopVideo();

    camRunning_ = false;
}

// ---------------------------------------------------------------------------
// Slots — mode toggle
// ---------------------------------------------------------------------------

void MainWindow::onPhotoModeClicked()
{
    if (mode_ == Mode::Photo) {
        photoModeBtn_->setChecked(true); // keep checked
        return;
    }
    if (recordingActive_.load()) {
        setStatus("Stop recording before switching modes.");
        videoModeBtn_->setChecked(true);
        return;
    }
    stopCamera();
    setMode(Mode::Photo);
    startCameraPhotoMode();
}

void MainWindow::onVideoModeClicked()
{
    if (mode_ == Mode::Video) {
        videoModeBtn_->setChecked(true);
        return;
    }
    stopCamera();
    setMode(Mode::Video);
    startCameraVideoMode();
}

// ---------------------------------------------------------------------------
// Slots — shutter
// ---------------------------------------------------------------------------

void MainWindow::onShutterClicked()
{
    if (mode_ == Mode::Photo)
        capturePhoto();
    else if (!recordingActive_.load())
        startRecording();
    else
        stopRecording();
}

void MainWindow::capturePhoto()
{
    shutterButton_->setEnabled(false);
    setStatus("Capturing…");

    // Run in a detached thread — capturePhoto() can block for several hundred ms.
    std::thread([this]() {
        cv::Mat photo;
        if (cam_.capturePhoto(photo)) {
            QString fname = timestampFilename("photo", ".jpg");
            cv::imwrite(fname.toStdString(), photo);
            QMetaObject::invokeMethod(this, [this, fname]() {
                setStatus("Saved: " + fname);
                shutterButton_->setEnabled(true);
            }, Qt::QueuedConnection);
        } else {
            QMetaObject::invokeMethod(this, [this]() {
                setStatus("Capture failed.");
                shutterButton_->setEnabled(true);
            }, Qt::QueuedConnection);
        }
    }).detach();
}

void MainWindow::startRecording()
{
    QString fname = timestampFilename("video", ".avi");
    recordingActive_.store(true);

    // Update UI immediately.
    recordingDot_->setVisible(true);
    recordingTime_->setVisible(true);
    recordingTime_->setText("00:00");
    shutterButton_->setStyleSheet(R"(
        QPushButton {
            background-color: #ff3030;
            border: 4px solid #cc0000;
            border-radius: 36px;
        }
        QPushButton:pressed { background-color: #cc0000; }
    )");
    recClock_.start();
    recTimer_->start();
    setStatus("Recording: " + fname);

    // Polling thread — reads high-resolution video frames and writes them.
    recordThread_ = std::thread([this, fname]() {
        cv::VideoWriter writer;
        while (recordingActive_.load()) {
            cv::Mat frame;
            if (cam_.getVideoFrame(frame, 100)) {
                if (!writer.isOpened()) {
                    writer.open(fname.toStdString(),
                                cv::VideoWriter::fourcc('M','J','P','G'),
                                static_cast<double>(cam_.options->framerate),
                                cv::Size(static_cast<int>(cam_.options->video_width),
                                         static_cast<int>(cam_.options->video_height)));
                    if (!writer.isOpened()) {
                        QMetaObject::invokeMethod(this, [this]() {
                            setStatus("Failed to open video writer.");
                        }, Qt::QueuedConnection);
                        recordingActive_.store(false);
                        return;
                    }
                }
                writer.write(frame);
            }
        }
        writer.release();
    });
}

void MainWindow::stopRecording()
{
    recordingActive_.store(false);
    if (recordThread_.joinable())
        recordThread_.join();

    recTimer_->stop();
    recordingDot_->setVisible(false);
    recordingTime_->setVisible(false);

    // Restore the video-mode shutter style.
    setMode(Mode::Video);
    setStatus("Recording saved.");
}

void MainWindow::tickRecordingTimer()
{
    qint64 ms  = recClock_.elapsed();
    int secs   = static_cast<int>(ms / 1000);
    int mins   = secs / 60;
    secs      %= 60;
    recordingTime_->setText(QString("%1:%2")
        .arg(mins, 2, 10, QChar('0'))
        .arg(secs, 2, 10, QChar('0')));
}

// ---------------------------------------------------------------------------
// Slots — zoom
// ---------------------------------------------------------------------------

void MainWindow::onZoomIn()
{
    float z = cam_.options->zoom + kZoomStep;
    if (z > kZoomMax) z = kZoomMax;
    cam_.options->zoom = z;
    // The dispatcher picks up zoom changes automatically on the next frame.
    updateZoomLabel();
}

void MainWindow::onZoomOut()
{
    float z = cam_.options->zoom - kZoomStep;
    if (z < kZoomMin) z = kZoomMin;
    cam_.options->zoom = z;
    updateZoomLabel();
}

void MainWindow::updateZoomLabel()
{
    zoomInButton_->setEnabled(cam_.options->zoom < kZoomMax);
    zoomOutButton_->setEnabled(cam_.options->zoom > kZoomMin);
    zoomLabel_->setText(QString("%1×").arg(
        static_cast<double>(cam_.options->zoom), 0, 'f', 2));
}

// ---------------------------------------------------------------------------
// Slots — settings
// ---------------------------------------------------------------------------

void MainWindow::onSettingsClicked()
{
    // Pause the camera briefly while the dialog is open.
    bool wasRunning = camRunning_;
    if (wasRunning) {
        if (recordingActive_.load()) {
            setStatus("Stop recording before opening settings.");
            return;
        }
        cam_.stopViewfinder();
    }

    SettingsDialog dlg(cam_.options, this);
    if (dlg.exec() == QDialog::Accepted) {
        // Options were updated inside the dialog.
        // Fully restart so new controls take effect.
        if (wasRunning) {
            if (mode_ == Mode::Photo) cam_.stopPhoto();
            else                      cam_.stopVideo();
            camRunning_ = false;
        }
    }

    if (wasRunning) {
        if (!camRunning_) {
            // Re-start after settings were applied.
            if (mode_ == Mode::Photo) startCameraPhotoMode();
            else                      startCameraVideoMode();
        } else {
            // Dialog was cancelled — just re-attach viewfinder.
            attachViewfinderCallback();
        }
    }
}

// ---------------------------------------------------------------------------
// Frame display
// ---------------------------------------------------------------------------

void MainWindow::updateFrame(QImage image)
{
    viewfinderLabel_->setPixmap(
        QPixmap::fromImage(image).scaled(
            viewfinderLabel_->size(),
            Qt::KeepAspectRatio,
            Qt::FastTransformation));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void MainWindow::setStatus(const QString &msg)
{
    statusLabel_->setText(msg);
}

QString MainWindow::timestampFilename(const QString &prefix, const QString &ext) const
{
    return prefix + "_" +
           QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") +
           ext;
}
