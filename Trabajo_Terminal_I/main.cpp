#include <cstdio>
#include <cmath>
#include <string>
#include <iostream>
#include <map>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "frame.h"
#include "lucas_kanade.h"
#include "video_factory.h"
#include "horn_schunck.h"
#include "simple_flow.h"
#include "test_generator.h"

//#define TAG_STRING "PIEH"

const int kUp = 0x00FFFF; // LIGHT BLUE
const int kDown = 0x00FF00; // GREEN
const int kLeft = 0xFF0000; // RED
const int kRight = 0xFFFF00; // YELLOW
const double kIntensity = 4.7;

double CalcAngularError(double u, double v, double gtu, double gtv) {
	return std::acos((1.0 + u * gtu + v * gtv) / (std::sqrt(1.0 + u * u + v * v) * std::sqrt(1.0 + gtu * gtu + gtv * gtv)));
}

double CalcEndpointError(double u, double v, double gtu, double gtv) {
	double du = u - gtu, dv = v - gtv;
	return std::sqrt(du * du + dv * dv);
}

typedef long long int lld;
lld pix_sum;
double ae_sum, ee_sum;
std::map< double, lld > ae_r, ee_r;

void InitStats(std::vector< int > ae_r_l, std::vector< int > ee_r_l) {
	pix_sum = 0;
	ae_sum = ee_sum = 0.0;
	for (int ae_r_v : ae_r_l) {
		ae_r[ae_r_v] = 0;
	}
	for (int ee_r_v : ee_r_l) {
		ee_r[ee_r_v] = 0;
	}
}

void CalcStats(cv::Mat u, cv::Mat v, cv::Mat gtu, cv::Mat gtv, cv::Mat mask) {
	int rows = u.rows, cols = v.cols;
	for (int i = 0; i < rows; ++i) {
		double* p_u = u.ptr<double>(i);
		double* p_v = v.ptr<double>(i);
		double* p_gtu = gtu.ptr<double>(i);
		double* p_gtv = gtv.ptr<double>(i);
		char* p_mask = mask.ptr<char>(i);
		for (int j = 0; j < cols; ++j, ++p_u, ++p_v, ++p_gtu, ++p_gtv, ++p_mask) {
			if (!(*p_mask)) {
				continue;
			}
			double ae = CalcAngularError(*p_u, *p_v, *p_gtu, *p_gtv);
			double ee = CalcEndpointError(*p_u, *p_v, *p_gtu, *p_gtv);
			ae_sum += ae;
			ee_sum += ee;
			for (auto ae_r_p = ae_r.begin(); ae_r_p != ae_r.end(); ae_r_p++) {
				if (ae >= ae_r_p->first) {
					ae_r_p->second++;
				}
			}
			for (auto ee_r_p = ee_r.begin(); ee_r_p != ee_r.end(); ee_r_p++) {
				if (ee >= ee_r_p->first) {
					ee_r_p->second++;
				}
			}
		}
	}
}


//read nameFile starting_number 
//write to flow10.flo

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cout << "Video input file and output directory required.";
		return 0;
	}

	cv::VideoCapture vcapture;

	vcapture.open(argv[1]);
	if (!vcapture.isOpened()) {
		std::cout << "Could not initialize capturing.\n";
		return 0;
	}

	cv::Mat capture;
	std::string dir = std::string(argv[1]);

	
	int width = 1920;
	int height = 1080;

	int orig_width;
	int orig_height;

	/*
	TestGenerator::GenerateTest("C:\\Users\\Adonais\\Desktop\\test.avi", 320, 240, 5);

	return 0;
	*/
	
	LucasKanade lk;

	VideoFactory lk_vf(dir + "-lk-flow.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

	cv::Mat vx, vy;
	std::cout << "\n\nStarting process.\n";
	int fps = (int)vcapture.get(CV_CAP_PROP_FPS);
	Frame* lk_result = new Frame(false);
	
	//int fileNumber = 7;
	//char* number = new char[3];
	
	for (int i = 0; i <= 30; ++i) {
		//sprintf(number, "%02d", fileNumber);
		//std::string fileName = std::string( argv[1] ) + std::string(number) + ".flo";
		//std::string imageName = std::string( argv[1] ) + std::string(number) + ".png";
		std::cout << "Processing frame " << i << ".\n";

		//capture = cv::imread(imageName);
		vcapture >> capture;
		if (capture.empty()) break;

		if (i == 0) {
			lk_result->SetMatrix(&capture);
			lk_result->Rescale(width, height);
			lk_result->GetMatrixOnCache();
		}

		Frame* frame = new Frame(&capture);

		orig_width = frame->Columns();
		orig_height = frame->Rows();

		//width = orig_width / 2;
		//height = orig_height / 2;

		frame->Rescale(width, height);
		frame->GetMatrixOnCache();
		lk.AddFrame(frame);

		lk.CalculateFlow(vx, vy);

		//Code for flo file
		//FILE *stream = fopen(fileName.c_str(), "wb");
		//puts(fileName.c_str());
		// write the header
		//fprintf(stream, TAG_STRING);
		//fwrite(&width,  sizeof(int),   1, stream);
		//fwrite(&height, sizeof(int),   1, stream);
		
		for (int x = 0; x < height; ++x) {
			double* ptr_vx = vx.ptr<double>(x);
			double* ptr_vy = vy.ptr<double>(x);
			for (int y = 0; y < width; ++y, ++ptr_vx, ++ptr_vy) {
				double X = *ptr_vx, Y = *ptr_vy;

				//code for flo file
				//float xF = (float)X;
				//float yF = (float)Y;

				// write the rows
				//fwrite(&xF, sizeof(float), 1, stream);
				//fwrite(&yF, sizeof(float), 1, stream);

				int hor_color = 0;
				double intensity = std::min(std::abs(Y) / kIntensity, 1.0);
				for (int k = 0; k <= 16; k += 8) {
					int color = (Y < 0)? (kLeft >> k) & 255: (kRight >> k) & 255;
					hor_color |= static_cast<int>(color * intensity) << k;
				}
				int ver_color = 0;
				intensity = std::min(std::abs(X) / kIntensity, 1.0);
				for (int k = 0; k <= 16; k += 8) {
					int color = (X < 0)? (kUp >> k) & 255: (kDown >> k) & 255;
					ver_color |= static_cast<int>(color * intensity) << k;
				}

				lk_result->SetPixel(x, y, hor_color | ver_color);
			}
		}
		//fclose(stream);
		lk_result->GetCacheOnMatrix();
		lk_vf.AddFrame(lk_result->GetMatrix());
	}
	
	
	/*
	HornSchunck hs;

	VideoFactory hs_vf(dir + "-hs-flow.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

	double* u = NULL, *v = NULL;
	Frame* hs_result = new Frame(false);

	int fps = (int)vcapture.get(CV_CAP_PROP_FPS);
	int i = 0;

	//std::cout << "\n\nStarting process.\n";
	for (i = 0; i < fps * 10; ++i) {
	//std::cout << "Processing frame " << i << ".\n";

	vcapture >> capture;
	if (capture.empty()) break;

	if (!i) {
	hs_result->SetMatrix(&capture);
	hs_result->Rescale(width, height);
	hs_result->GetMatrixOnCache();
	}

	Frame* frame = new Frame(&capture);

	orig_width = frame->Columns();
	orig_height = frame->Rows();

	frame->Rescale(width, height);
	frame->GetMatrixOnCache();
	hs.AddFrame(frame);

	if (i % 1 == 0) {
	delete [] u;
	delete [] v;
	hs.CalculateFlow(&u, &v);
	}

	int rows = frame->Rows();
	int cols = frame->Columns();
	double* ptr_x = u, *ptr_y = v;
	for (int x = 0; x < rows; ++x) {
	for (int y = 0; y < cols; ++y, ++ptr_x, ++ptr_y) {
	double X = *ptr_x, Y = *ptr_y;

	int hor_color = 0;
	double intensity = std::min(std::abs(Y) / kIntensity, 1.0);
	for (int k = 0; k <= 16; k += 8) {
	int color = (Y < 0)? (kLeft >> k) & 255: (kRight >> k) & 255;
	hor_color |= static_cast<int>(color * intensity) << k;
	}
	int ver_color = 0;
	intensity = std::min(std::abs(X) / kIntensity, 1.0);
	for (int k = 0; k <= 16; k += 8) {
	int color = (X < 0)? (kUp >> k) & 255: (kDown >> k) & 255;
	ver_color |= static_cast<int>(color * intensity) << k;
	}

	hs_result->SetPixel(x, y, hor_color | ver_color);
	}
	}

	hs_result->GetCacheOnMatrix();
	hs_vf.AddFrame(hs_result->GetMatrix());
	}
	printf("%d %d\n", i, fps); 
	printf("Ancho * alto = %d %d\n", orig_width, orig_height);
	*/
	/*
	SimpleFlow hs;

	VideoFactory hs_vf(dir + "-sf-flow.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

	int fps = (int)vcapture.get(CV_CAP_PROP_FPS);
	int i = 0;

	cv::Mat u, v;
	Frame* hs_result = new Frame(false);
	//std::cout << "\n\nStarting process.\n";
	for (i = 0; i < 10 * fps; ++i) {
		//std::cout << "Processing frame " << i << ".\n";

		vcapture >> capture;
		if (capture.empty()) break;

		if (!i) {
			hs_result->SetMatrix(&capture);
			hs_result->Rescale(width, height);
			hs_result->GetMatrixOnCache();
		}

		Frame* frame = new Frame(&capture);

		orig_width = frame->Columns();
	orig_height = frame->Rows();

		frame->Rescale(width, height);
		frame->GetMatrixOnCache();
		hs.AddFrame(frame);

		if (i % 1 == 0) {
			hs.CalculateFlow(u, v);
		}

		if (i == 0) {
			continue;
		}

		int rows = u.rows;
		int cols = u.cols;

		for (int x = 0; x < rows; ++x) {
			double* ptr_x = u.ptr<double>(x);
			double* ptr_y = v.ptr<double>(x);
			for (int y = 0; y < cols; ++y, ++ptr_x, ++ptr_y) {
				double X = *ptr_x, Y = *ptr_y;

				int hor_color = 0;
				double intensity = std::min(std::abs(Y) / kIntensity, 1.0);
				for (int k = 0; k <= 16; k += 8) {
					int color = (Y < 0) ? (kLeft >> k) & 255 : (kRight >> k) & 255;
					hor_color |= static_cast<int>(color * intensity) << k;
				}
				int ver_color = 0;
				intensity = std::min(std::abs(X) / kIntensity, 1.0);
				for (int k = 0; k <= 16; k += 8) {
					int color = (X < 0) ? (kUp >> k) & 255 : (kDown >> k) & 255;
					ver_color |= static_cast<int>(color * intensity) << k;
				}

				hs_result->SetPixel(x, y, hor_color | ver_color);
			}
		}

		hs_result->GetCacheOnMatrix();
		hs_vf.AddFrame(hs_result->GetMatrix());
	}
	printf("%d %d\n", i, fps); 
	printf("Ancho * alto = %d %d\n", orig_width, orig_height);
	*/
	return 0;
}
