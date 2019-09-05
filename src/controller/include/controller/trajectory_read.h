#ifndef _TRAJECTORY_READ_H_
#define _TRAJECTORY_READ_H_

#include "ros/ros.h"
#include "controller/trajectory.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>


void trajectory_simdata_read(std::vector<Trajectory> * csvdata);







#endif 