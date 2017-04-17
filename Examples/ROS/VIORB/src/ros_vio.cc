/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"../../../include/System.h"

#include <boost/foreach.hpp>
#include <fstream>
#include <time.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>

//#include "Eigen/Dense"
#include "imudata.h"
#include "configparam.h"
#include "imuintegrator.h"

using namespace std;
//using namespace Eigen;

typedef struct ImageList
{
	unsigned long timeStamp;
	string imgName;
}ICell;

void UpdateNoORB()
{

}
void loadImageList(char * imagePath,std::vector<ICell> &iListData)
{
    ifstream inf;
    inf.open(imagePath, ifstream::in);
    const int cnt = 2;          // csv 列数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;
    ICell temp;
    getline(inf,line);
    while (!inf.eof())
    {
        getline(inf,line);

        comma = line.find(',',0);
        temp.timeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());
        
        while (comma < line.size() && j != cnt-1)
        {   
            comma2 = line.find(',',comma + 1);
            temp.imgName = line.substr(comma + 1,comma2-comma-1).c_str();
            ++j;
            comma = comma2;
        }
        iListData.push_back(temp);
        j = 0;
    }

    inf.close();

    cout<<"IMG size: "<<iListData.size()<<endl;
    cout<<"IMG capacity: "<<iListData.capacity()<<endl;
}


void loadIMUFile(char * imuPath,std::vector<IMUData> &vimuData)
{
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;          // csv 列数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    double acc[3] = {0.0};
    double grad[3] = {0.0};
    unsigned long imuTimeStamp = 0;

    getline(inf,line);
    while (!inf.eof())
    {
        getline(inf,line);

        comma = line.find(',',0);
	string temp = line.substr(0,comma);
        imuTimeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());

        while (comma < line.size() && j != cnt-1)
        {
	   
            comma2 = line.find(',',comma + 1);
            switch(j)
	    {
              case 0:
		grad[0] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
              case 1:
		grad[1] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
              case 2:
		grad[2] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
              case 3:
		acc[0] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
              case 4:
		acc[1] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
              case 5:
		acc[2] = atof(line.substr(comma + 1,comma2-comma-1).c_str());

		break;
            }
            ++j;
            comma = comma2;
        }
        IMUData tempImu(grad[0],grad[1],grad[2],acc[0],acc[1],acc[2],imuTimeStamp);
        vimuData.push_back(tempImu);

        j = 0;

    }

inf.close();
    cout<<"IMU size: "<<vimuData.size()<<endl;
    cout<<"IMU capacity: "<<vimuData.capacity()<<endl;

}

int main(int argc, char **argv)
{

    if(argc != 6)
    {
        cerr << endl << "Usage: ./project path_to_ORBVOC.TXT path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data" << endl;
        return 1;
    }



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
   // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ConfigParam config(argv[2]);



    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    Matrix3d Rci = config.GetEigTbc().block<3,3>(0,0);
    Vector3d Tci = config.GetEigTbc().block<3,1>(0,3);
    cout<<"-------------------param got!---------------------------------------------"<<endl;
    char *fullPath = new char[500];
    memset(fullPath,0,500);

    std::vector<IMUData> allimuData;
    std::vector<IMUData> vimuData;
    std::vector<ICell> iListData;

    //load IMU
    loadIMUFile(argv[3],allimuData);
    cout<<"----------loading imu finished----------"<<endl;
    vimuData = allimuData;
    vimuData[0]._t=0;
    double e = pow(10.0,-9);
    for(int m=1;m<vimuData.size();m++)
    {
         /* if(bAccMultiply98)
        {
            allimuData[m]._a(0) *= g3dm;
            allimuData[m]._a(1) *= g3dm;
            allimuData[m]._a(2) *= g3dm;
        }*/

        vimuData[m]._t = (allimuData[m]._t-allimuData[m-1]._t)*e;   //dt
    }
    cout<<"---------------vimuData got--------------"<<endl;

    //load IMG
    loadImageList(argv[4],iListData);
    cout<<"---------------loading image finished-----"<<endl;


    IMUIntegrator EKF;
return 0;

 //   for(int j=0;j<iListData.size();j++)
    for(int j=0;j<30;j++)
    {

	    /*
	    *imu 的频率是200HZ 图像帧率是20HZ 所以简单的认为每一帧图像对应10个imu数据
	    */
            for(unsigned int i=0;i<10;i++)
            {
		/*
		*这里将时间戳×上e-9后的结果，程序可以正常运行，但是显示出来的时间和ros环境下的时间不同，且运行速度缓慢。
		ORB_SLAM2::IMUData imudata(allimuData[j]._g(0),allimuData[j]._g(1),allimuData[j]._g(2),
                                allimuData[j]._a(0),allimuData[j]._a(1),allimuData[j]._a(2),(double)allimuData[j]._t);
		*/
		//时间戳按这个给程序也可以正常运行，速度基本和ros环境下一样，问题在于当按照正常的0.005设置时候程序会挂，后来尝试后发现这个数据给的越小程序越容易运行成功。
//                IMUData imudata(allimuData[10*j+i]._g(0),allimuData[10*j+i]._g(1),allimuData[10*j+i]._g(2),
//                               // allimuData[10*j+i]._a(0),allimuData[10*j+i]._a(1),allimuData[10*j+i]._a(2),j*0.0005+i*0.00005);
//                                 allimuData[10*j+i]._a(0),allimuData[10*j+i]._a(1),allimuData[10*j+i]._a(2),allimuData[10*j+i]._t);
//                vimuData.push_back(imudata);

                EKF.updateNOv(vimuData[10*j+i]._g, vimuData[10*j+i]._a, vimuData[10*j+i]._t);

            }

            //cout<<"IMU FINISHED READING"<<endl;
	    //发现读取txt时，图像文件名后多了一个‘/r’，因此需要截掉这个字符。
	    string temp = iListData[j].imgName.substr(0,iListData[j].imgName.size()-1);

        sprintf(fullPath,"%s/%s",argv[5],temp.c_str());
	    cv::Mat im = cv::imread(fullPath,0);
	    memset(fullPath,0,100);
	    iListData[j].timeStamp = iListData[j].timeStamp*e;
        // Pass the image to the SLAM system
        Matrix4d eigenT = Matrix4d::Identity();
        cv::Mat TfromORB(4,4,CV_32F);
/*
        TfromORB = SLAM.TrackMonocular(im,(double)iListData[j].timeStamp).clone();
//        Eigen::Map<Matrix4f> eigenT( TfromORB.ptr<float>(), TfromORB.rows,TfromORB.cols );

        if(TfromORB.empty())
        {
            //
        }
        else
        {
        if(!TfromORB.empty())   //不加的话会段存储错误
        {
        for(int _i=0;_i<4;_i++)
            for(int _j=0;_j<4;_j++)
            {
                eigenT(_i,_j) = TfromORB.at<float>(_i,_j) ;
            }
        }
        cout<<eigenT<<endl;
        cout<<TfromORB<<endl;
        }

*/
	    //printf("imagetimestamp,%0.10f\n",(double)iListData[j].timeStamp);
	    //SLAM.TrackMonoVI(im, vimuData, (double)iListData[j].timeStamp);
        //    SLAM.TrackMonoVI(im, vimuData, j*0.00005);
		//if(j == 6)
		//{
		//	usleep(20);			
			//cv::waitKey(0);
		//}
            // Wait local mapping end.
         //   bool bstop = false;
	//cout<<"----------------------------------"<<j<<"----------------------------------------"<<endl;
//            while(!SLAM.bLocalMapAcceptKF())
//            {
//               bstop=true;
//            };
            //if(bstop)
              //  break;
     }
    delete [] fullPath;
   // SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
  //  SLAM.Shutdown();



    //return 0;
}


