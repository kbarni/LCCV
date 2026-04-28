/*
 * qt_smartphone — LCCV Qt camera demo
 *
 * A smartphone-style camera application using the LCCV library and Qt Widgets.
 *
 * Layout:
 *   Left area : live viewfinder
 *   Right toolbar (top to bottom):
 *       Photo / Video mode buttons
 *       Shutter button
 *       Zoom level label + Zoom In / Out buttons
 *       Settings button
 *
 * Photo mode  — press Shutter to capture a JPEG still.
 * Video mode  — press Shutter to start/stop recording (MJPEG AVI).
 * Zoom        — zoom in/out using + / - buttons (1×–4×).
 * Settings    — EV, white-balance, and metering controls.
 *
 * Files are saved in the current working directory with a timestamp name.
 */

#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("LCCV Camera");
    app.setOrganizationName("LCCV");

    MainWindow window;
    window.setWindowTitle("LCCV Camera");
    window.show();

    return app.exec();
}
