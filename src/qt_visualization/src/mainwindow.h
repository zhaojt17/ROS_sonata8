#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QTimer>
#include <QtWebEngineWidgets>
#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"

namespace Ui {
class MainWindow;
}




class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void positionCallback(const geometry_msgs::Pose2D &pose);

    QWebEngineView *engine;

private:
    Ui::MainWindow *ui;

private slots:
    void onTimerOut();
	void loadFinished(bool a);
};

#endif // MAINWINDOW_H
