#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <iostream>


std::string qstr2str(const QString qstr)  
{  
    QByteArray cdata = qstr.toLocal8Bit();  
    return std::string(cdata);  
}  

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // ROS 
    ROS_INFO("Start to receive position");

    // TIMER 
    QTimer* timer = new QTimer(this);
    timer->setInterval(500);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimerOut()));
    timer->start();

    //JS
    engine = new QWebEngineView();

    connect(engine, SIGNAL(loadFinished(bool)), this, SLOT(loadFinished(bool)));
   
	QUrl url = QUrl("file:///home/zzh/Desktop/Rosws/qtws/src/qt_visualization/html/display.html");
    


    engine -> load(url);
    engine -> show();

     
}

void MainWindow::loadFinished(bool a)
{
	if (a == true)
	{
		
	}	
}



MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::onTimerOut()
{
    ros::spinOnce();
}

void MainWindow::positionCallback(const geometry_msgs::Pose2D &pose)
{
	ROS_INFO("x = %f ", pose.x);

    static int i = 0;
    QString command;

        if (i == 0){ command = QString("node_display(116.330435, 39.99967911,0)");}
        if (i == 1){command = QString("node_display(116.332440, 39.99967911,0)");}
        if (i == 2){ command = QString("node_display(116.334445, 39.99967911,0)");}
        if (i == 3){  command = QString("node_display(116.336450, 39.99967911,0)");}
        i++;
    if (i == 4) {i=0;}

    engine -> page() -> runJavaScript(command);
    
}

