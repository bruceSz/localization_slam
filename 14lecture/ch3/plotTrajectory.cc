/***************************************************************************
 *
 * Copyright (c) 2022 Baidu.com, Inc. All Rights Reserved
 * $Id$
 *
 **************************************************************************/



/**
 * @file plotTrajectory.cc
 * @author songjian02(songjian02@baidu.com)
 * @date 2022/02/14 10:14:55
 * @version $Revision$
 * @brief
 *
 **/






#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <unistd.h>

using namespace std;
using namespace Eigen;

string traject_file = "./example/trajectory.txt"

void DrawJrajector(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);


int main(int argc, char** argv) {
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(traject_file);
    if (! fin) {
        cout << "open trajectory file failed: " << traject_file << endl;
        return 1;
    }

    while(!fin.eof()) {
        double time, tx, ty, tz, 
    }
}
















/* vim: set ts=4 sw=4 sts=4 tw=100: */
