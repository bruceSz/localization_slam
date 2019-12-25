
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System_sim.h"
#include "param.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

sim::Param params;

std::shared_ptr<sim::System> pSystem;

void PubImuData()
{
	string sImu_data_file = sData_path + "imu_pose.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		exit(1);
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	cout << "begin to getline for imu_pose." << std::endl;
	int num = 0;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);

		//sim data format:
		// 			<<time<<" "
        //           <<q.w()<<" "
        //           <<q.x()<<" "
        //           <<q.y()<<" "
        //           <<q.z()<<" "
        //           <<t(0)<<" "
        //           <<t(1)<<" "
        //           <<t(2)<<" "
        //           <<gyro(0)<<" "
        //           <<gyro(1)<<" "
        //           <<gyro(2)<<" "
        //           <<acc(0)<<" "
        //           <<acc(1)<<" "
        //           <<acc(2)<<" "

		double qx, qy, qz, qw, tx, ty, tz;
		ssImuData >> dStampNSec  >> qw >> qx >> qy >> qz >> tx >> ty >> tz
		>> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;

		pSystem->PubImuData(dStampNSec , vGyr, vAcc);
		auto milli = 1000/params.imu_frequency ;
		usleep(milli*1000*nDelayTimes);
		num++;
		cout << "imu with ts " << dStampNSec << " published ." << std::endl;

		if (num %100 == 0) {
			cout << "num 100 imu measurement." << std::endl;
		}
	}
	fsImu.close();
}


bool get_points_info(std::string prefix, int num, vector<sim::SIM_PTS_INFO>& fts) {
	std::string file = prefix + "all_points_" + std::to_string(num) + ".txt";
	ifstream fsPoint;
	fsPoint.open(file.c_str());
	if (!fsPoint.is_open())
	{
		cerr << "Failed to open keyframe point file! " << file << endl;
		return false ;
	}

	std::string points_line;
	while (std::getline(fsPoint, points_line) && !points_line.empty()) // read imu data
	{
		std::istringstream ssPointData(points_line);

		sim::SIM_PTS_INFO pts_info;
		Eigen::Vector4d p;
		Eigen::Vector2d ft;
		double ts;
		ssPointData >> ts >>  p(0) >> p(1) >> p(2) >> p(3) >> ft(0) >> ft(1);
		pts_info.ft = ft;
		pts_info.point = p;
		pts_info.ts = ts;
		
		fts.push_back(pts_info);
	}
	fsPoint.close();
	return true;
}


void PubImageData()
{
	// sim data has no image , just some feature points.
	// rewrite this.
	string sKeyFrame = sData_path + "keyframe/";

	cout << "1 PubImageData start keyframe dir: " << sKeyFrame << endl;

	
	// t here is the imu timestamp.
	// t also the camera timestamp.
	// n is the index of of this timestamp;
	int n = 0;
	int milli =  1000/params.cam_frequency;
	for (float t = params.t_start; t<params.t_end;) {
		std::vector<sim::SIM_PTS_INFO> fts;
		if (!get_points_info(sKeyFrame,n, fts))  {
			continue;
		}
		cout << "all " << fts.size() << " fts for camera at ts: " << t << std::endl;
		pSystem->PubImageFts(t, fts);
	    t += 1.0/params.cam_frequency;
		n++;

		usleep(milli * 1000*nDelayTimes);
	}
	exit(1);
}

#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

} 
#endif

int main(int argc, char **argv)
{
	if(argc != 3)
	{
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
		return -1;
	}
	sData_path = argv[1];
	sConfig_path = argv[2];

	pSystem.reset(new sim::System(sConfig_path, &params));
	
	std::thread thd_BackEnd(&sim::System::ProcessBackEnd, pSystem);
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

#ifdef __linux__	
	std::thread thd_Draw(&sim::System::Draw, pSystem);
#elif __APPLE__
	DrawIMGandGLinMainThrd();
#endif

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
#ifdef __linux__	
	thd_Draw.join();
#endif

	cout << "main end... see you ..." << endl;
	return 0;
}
