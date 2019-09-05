#include "controller/trajectory_read.h"
using namespace std;
void trajectory_simdata_read(std::vector<Trajectory> * csvdata)
{
    FILE *fp;
    Trajectory data;
    fp=fopen("/home/zhaojt/ROS/test.csv","r");
    //ROS_INFO("FILE OPEN");
    while(1)
    {
        fscanf(fp,"%d,%f,%f,%d,%f\n",&data.num,&data.x,&data.y,&data.status,&data.speed);
        csvdata->push_back(data);
        if (feof(fp))break;
    }
}
