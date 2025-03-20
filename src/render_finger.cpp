////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include <QApplication>
#include <QWidget>
#include <QPainter>
#include "include/render_finger.hpp" // Include the header file

////////////////////////////////////////////////////////////
/// Constants
////////////////////////////////////////////////////////////
#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600

class FingerWidget : public QWidget
{
protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Clear the window color to white
        painter.fillRect(rect(), Qt::white);

        // Draw Base Link
        painter.setBrush(Qt::red);
        painter.save();
        painter.translate(320, 450);
        painter.rotate(90);
        painter.drawRect(-75, -20, 150, 40);
        painter.restore();

        // Draw Link1
        painter.setBrush(Qt::red);
        painter.save();
        painter.translate(300, 430);
        painter.drawRect(-75, -20, 150, 40);
        painter.restore();

        // Draw Joint1
        painter.setBrush(Qt::blue);
        painter.save();
        painter.translate(270, 420);
        painter.drawEllipse(-30, -30, 60, 60);
        painter.restore();
    }
};

void render_finger()
{
    int argc = 0;
    char *argv[] = {nullptr};
    QApplication app(argc, argv);

    FingerWidget window;
    window.resize(WINDOW_WIDTH, WINDOW_HEIGHT);
    window.setWindowTitle("Qt Finger");
    window.show();

    app.exec();
}
