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

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <System.h>
#include <MapPoint.h>

using namespace std;

extern "C" 
{
	__declspec(dllexport) void* create_SLAM_system
	(
		const char* vocabulary_file_path, 
		const char* camera_config_file,
		unsigned char display_window
	)
	{
		ORB_SLAM2::System* slam_ptr = 
			new ORB_SLAM2::System
			(
				vocabulary_file_path, 
				camera_config_file, 
				ORB_SLAM2::System::MONOCULAR, 
				display_window != 0
			);

		return slam_ptr;
	}

	__declspec(dllexport) void reset_slam_system(void* slam_system_ptr)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;
		slam_system->Reset();
	}

	__declspec(dllexport) void activate_localization_mode(void* slam_system_ptr)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;
		slam_system->ActivateLocalizationMode();
	}

	__declspec(dllexport) void deactivate_localization_mode(void* slam_system_ptr)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;
		slam_system->DeactivateLocalizationMode();
	}

	__declspec(dllexport) void shutdown_slam_system(void* slam_system_ptr)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;
		slam_system->Shutdown();
	}

	__declspec(dllexport) void delete_pointer(void* pointer)
	{
		delete(pointer);
	}

	__declspec(dllexport) void free_pointer(void* pointer)
	{
		free(pointer);
	}

	__declspec(dllexport) float* update_image
	(
		void* slam_system_ptr, 
		unsigned char* image_data, 
		int width, 
		int height, 
		double time_stamp
	)
	{
		ORB_SLAM2::System* slam_system =  (ORB_SLAM2::System*) slam_system_ptr;
		cv::Mat frame(height, width, CV_8UC3, image_data);

		cv::Mat pose = slam_system->TrackMonocular(frame, time_stamp);

		if (pose.empty())
		{
			return nullptr;
		}
		else
		{
			float* matrix = new float[16];
			matrix[0] = pose.at<float>(0, 0);
			matrix[1] = pose.at<float>(0, 1);
			matrix[2] = pose.at<float>(0, 2);
			matrix[3] = pose.at<float>(0, 3);

			matrix[4] = pose.at<float>(1, 0);
			matrix[5] = pose.at<float>(1, 1);
			matrix[6] = pose.at<float>(1, 2);
			matrix[7] = pose.at<float>(1, 3);

			matrix[8] = pose.at<float>(2, 0);
			matrix[9] = pose.at<float>(2, 1);
			matrix[10] = pose.at<float>(2, 2);
			matrix[11] = pose.at<float>(2, 3);

			matrix[12] = pose.at<float>(3, 0);
			matrix[13] = pose.at<float>(3, 1);
			matrix[14] = pose.at<float>(3, 2);
			matrix[15] = pose.at<float>(3, 3);

			return matrix;
		}
	}

	__declspec(dllexport) float* get_tracked_screen(void* slam_system_ptr, int* size)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;
		*size = 0;
		auto key_points = slam_system->GetTrackedKeyPointsUn();
		auto vector_size = key_points.size();
		if (vector_size > 0)
		{
			float* points = new float[vector_size * 2];
			for (int i = 0; i < vector_size; i++)
			{
				auto current_point = key_points[i];
				points[i * 2 + 0] = current_point.pt.x;
				points[i * 2 + 1] = current_point.pt.y;
			}
			*size = vector_size * 2;
			return points;
		}
		else
		{
			return NULL;
		}
	}

	__declspec(dllexport) float* get_3d_tracked_points(void* slam_system_ptr, int* size)
	{
		ORB_SLAM2::System* slam_system = (ORB_SLAM2::System*) slam_system_ptr;

		auto vpMPs = slam_system->GetAllMapPoints();
		auto vpRefMPs = slam_system->GetReferenceMapPoints();

		set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

		*size = 0;
		if (vpMPs.empty())
		{
			return NULL;
		}

		int total_points = 0;
		for (size_t i = 0, iend = vpMPs.size(); i<iend; i++)
		{
			if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
			{
				continue;
			}
			total_points++;
		}

		for (auto sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
		{
			if ((*sit)->isBad())
			{
				continue;
			}
			total_points++;
		}

		*size = total_points;
		float* points = new float[total_points * 4];
		int index = 0;
		for (size_t i = 0, iend = vpMPs.size(); i<iend; i++)
		{
			if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
			{
				continue;
			}
			cv::Mat mat = vpMPs[i]->GetWorldPos();
			points[index * 4 + 0] = mat.at<float>(0, 0);
			points[index * 4 + 1] = mat.at<float>(0, 1);
			points[index * 4 + 2] = mat.at<float>(0, 2);
			points[index * 4 + 3] = 0;
			index++;
		}

		for (auto sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
		{
			if ((*sit)->isBad())
			{
				continue;
			}
			cv::Mat mat = (*sit)->GetWorldPos();
			points[index * 4 + 0] = mat.at<float>(0, 0);
			points[index * 4 + 1] = mat.at<float>(0, 1);
			points[index * 4 + 2] = mat.at<float>(0, 2);
			points[index * 4 + 3] = 1;
			index++;
		}
		return points;
	}
}


//int main(int argc, char **argv)
//{
//	cout << "starting" << endl;
//	if (argc != 3)
//	{
//		cerr << endl << "Usage: ./slam path_to_vocabulary path_to_camera_calibration_settings" << endl;
//		return 1;
//	}
//
//	cv::VideoCapture cap;
//
//	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//
//	// open the default camera, use something different from 0 otherwise;
//	// Check VideoCapture documentation.
//	if (!cap.open(0))
//	{
//		cout << "Cannot open webcam. Exiting." << endl;
//		return 0;
//	}
//
//	//cout << "CV_8UC3 is " << CV_8UC3 << endl;
//	cout << "create slam system" << endl;
//	// Create SLAM system. It initializes all system threads and gets ready to process frames.
//	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false);
//	cout << "slam system created." << endl;
//	cout << endl << "-------" << endl;
//
//	// Main loop
//	cv::Mat im;
//	double tframe = 0;
//	while (true)
//	{
//		cap >> im;
//
//		if (im.empty())
//		{
//			continue;
//		}
//
//		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//
//		auto ms = std::chrono::duration_cast<std::chrono::milliseconds >(
//			std::chrono::system_clock::now().time_since_epoch()
//		);
//
//		tframe = ms.count();
//		// Pass the image to the SLAM system
//		{
//			cv::Size s = im.size();
//			auto rows = s.height;
//			auto cols = s.width;
//
//			auto matrix_type = im.type();
//
//			cout << "nb rows : " << rows << " nb cols : " << cols << " type is " << matrix_type << endl;
//
//		}
//		auto pose = SLAM.TrackMonocular(im, tframe);
//
//		/*{
//			cv::Size s = pose.size();
//			auto rows = s.height;
//			auto cols = s.width;
//
//			auto matrix_type = pose.type();
//
//			cout << "nb rows : " << rows << " nb cols : " << cols << " type is " << matrix_type << endl;
//		}*/
//		cout << "M = " << endl << " " << pose << endl << endl;
//
//		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//
//		//double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//	}
//
//	// Stop all threads
//	SLAM.Shutdown();
//
//	// Save camera trajectory
//	//SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//
//	return 0;
//}