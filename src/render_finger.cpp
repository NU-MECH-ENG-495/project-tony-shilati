#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QKeyEvent>
#include <math.h>
#include "include/render_finger.hpp" // Include the header file

////////////////////////////////////////////////////////////
/// Constants
////////////////////////////////////////////////////////////
#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600

class FingerWidget : public QWidget
{
public:
    FingerWidget(QWidget *parent = nullptr)
        : QWidget(parent), base_angle(0), link1_angle(45), link2_angle(30), link3_angle(15)
    {
        // Set up a timer to update the positions periodically
        QTimer *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &FingerWidget::updatePositions);
        timer->start(100); // Update every 100 ms
    }

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
        painter.translate(0, 400);
        painter.drawRect(0, 20, 150, 40);
        painter.restore();

        // Draw Link1
        painter.setBrush(Qt::red);
        painter.save();
        painter.translate(150, 400);

        painter.rotate(link1_angle);
        painter.drawRect(0, 20, 150, 40);
        painter.restore();

        // Draw Joint1
        painter.setBrush(Qt::blue);
        painter.save();
        painter.translate(150, 380);

        painter.drawEllipse(-30, 30, 60, 60);
        painter.restore();

        // Draw Link2
        painter.setBrush(Qt::red);
        painter.save();
        painter.translate(150, 440);
        painter.rotate(link2_angle);
        painter.drawRect(0, -20, 150, 40);
        painter.restore();

        // Draw Joint2
        painter.setBrush(Qt::blue);
        painter.save();
        painter.translate(150, 380);
        painter.rotate(link1_angle);
        painter.translate(150, 0);
        painter.rotate(link2_angle);

        painter.drawEllipse(-30, -30, 60, 60);
        painter.restore();

        // Draw Link3
        painter.setBrush(Qt::red);
        painter.save();
        painter.translate(450, 440);
        painter.rotate(link3_angle);
        painter.drawRect(0, -20, 150, 40);
        painter.restore();

        // Draw Joint3
        painter.setBrush(Qt::blue);
        painter.save();
        painter.translate(450, 440);
        painter.drawEllipse(-30, -30, 60, 60);
        painter.restore();
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (event->key() == Qt::Key_P)
        {
            // Increment angles when 'P' is pressed
            updatePositions();
        }
        else if (event->key() == Qt::Key_E)
        {
            // Exit application when 'E' is pressed
            QApplication::quit();
        }
    }

private slots:
    void updatePositions()
    {
        // Update the angles to move the shapes
        base_angle += 1;
        link1_angle += 1;
        link2_angle += 1;
        link3_angle += 1;

        // Trigger a repaint
        update();
    }

private:
    int base_angle;
    int link1_angle = 0;
    int link2_angle = 0;
    int link3_angle = 0;

    int link1_length = 150;
    int link2_length = 150;
    int link3_length = 150;

    int J1[2] = {0, 0};
    int J2[2] = {0, 0};
    int J3[2] = {0, 0};
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