#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QElapsedTimer>

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <thread>

class SettingsDialog;

// ---------------------------------------------------------------------------
// MainWindow
// ---------------------------------------------------------------------------

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

Q_SIGNALS:
    // Emitted on the LCCV dispatcher thread; connected via QueuedConnection
    // so the slot runs on the main (GUI) thread.
    void frameReady(QImage image);

private Q_SLOTS:
    void updateFrame(QImage image);
    void onPhotoModeClicked();
    void onVideoModeClicked();
    void onShutterClicked();
    void onZoomIn();
    void onZoomOut();
    void onSettingsClicked();
    void tickRecordingTimer();

private:
    enum class Mode { Photo, Video };

    // UI construction
    void buildUi();
    void applyStyleSheet();

    // Camera lifecycle
    void startCameraPhotoMode();
    void startCameraVideoMode();
    void stopCamera();
    void attachViewfinderCallback();

    // Shutter actions
    void capturePhoto();
    void startRecording();
    void stopRecording();

    // Helpers
    void setMode(Mode m);
    void updateZoomLabel();
    void setStatus(const QString &msg);
    QString timestampFilename(const QString &prefix, const QString &ext) const;

    // ── UI widgets ──────────────────────────────────────────────────────────
    QLabel       *viewfinderLabel_;
    QPushButton  *photoModeBtn_;
    QPushButton  *videoModeBtn_;
    QLabel       *recordingDot_;    // red dot shown while recording
    QLabel       *recordingTime_;   // MM:SS counter
    QPushButton  *shutterButton_;
    QLabel       *zoomLabel_;       // "1.0×"
    QPushButton  *zoomInButton_;
    QPushButton  *zoomOutButton_;
    QPushButton  *settingsButton_;
    QLabel       *statusLabel_;
    QTimer       *recTimer_;
    QElapsedTimer recClock_;

    // ── Camera ──────────────────────────────────────────────────────────────
    lccv::Camera cam_;
    Mode         mode_    = Mode::Photo;
    bool         camRunning_ = false;

    // Video recording
    std::atomic<bool> recordingActive_{false};
    std::thread       recordThread_;

    static constexpr float kZoomStep = 0.25f;
    static constexpr float kZoomMin  = 1.0f;
    static constexpr float kZoomMax  = 4.0f;
};
