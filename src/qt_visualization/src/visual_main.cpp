#include "qt_visualization/visual_main.h"
#include "mainwindow.h"
// // #include <QWebEngineView>






int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    ros::init(argc, argv, "qt_visulizaiton");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("position", 1000, &MainWindow::positionCallback, &w);
    

    


    // view -> setUrl(QUrl("http://www.baidu.com"));
    // view -> show();

    
    
    // w.show();
    a.exec();
   
   
    // ros::spin();
    

}