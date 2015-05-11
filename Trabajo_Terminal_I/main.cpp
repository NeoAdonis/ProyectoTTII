#include <cstdio>
#include <cmath>
#include <cstdlib>
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

#define TAG_STRING "PIEH"

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
typedef std::map< double, lld > RX;

void CountMeasuresAboveThreshold(double err, RX &r) {
	for (auto r_p = r.begin(); r_p != r.end(); r_p++) {
		if (err >= r_p->first)
			r_p->second++;
	}
}


lld pix_sum;
double ae_sum, ee_sum;
RX ae_r, ee_r;

const double HIST_MUL = 1e6;
std::map< lld, lld > ae_hist, ee_hist;

void InitStats(std::vector< double > ae_r_l, std::vector< double > ee_r_l) {
	pix_sum = 0;
	ae_sum = ee_sum = 0.0;
	for (double ae_r_v : ae_r_l) {
		ae_r[ae_r_v] = 0;
	}
	for (double ee_r_v : ee_r_l) {
		ee_r[ee_r_v] = 0;
	}
}

void CalcStats(cv::Mat &u, cv::Mat &v, cv::Mat &gtu, cv::Mat &gtv, cv::Mat &mask, cv::Mat &aem, cv::Mat &eem) {
	int rows = u.rows, cols = v.cols;
	for (int i = 0; i < rows; ++i) {
		double* p_u = u.ptr<double>(i);
		double* p_v = v.ptr<double>(i);
		double* p_gtu = gtu.ptr<double>(i);
		double* p_gtv = gtv.ptr<double>(i);
		uchar* p_mask = mask.ptr<uchar>(i);
		double* p_aem = aem.ptr<double>(i);
		double* p_eem = eem.ptr<double>(i);
		for (int j = 0; j < cols; ++j, ++p_u, ++p_v, ++p_gtu, ++p_gtv, ++p_mask, ++p_aem, ++p_eem) {
			if (!(*p_mask)) {
				continue;
			}
			/*
			if (*p_gtu == 0.0 && *p_gtv == 0.0) {
				continue;
			}*/
			pix_sum++;
			double ae = CalcAngularError(*p_u, *p_v, *p_gtu, *p_gtv);
			double ee = CalcEndpointError(*p_u, *p_v, *p_gtu, *p_gtv);
			*p_aem = ae;
			*p_eem = ee;
			ae_sum += ae;
			ee_sum += ee;
			CountMeasuresAboveThreshold(ae, ae_r);
			CountMeasuresAboveThreshold(ee, ee_r);
			ae_hist[ae * HIST_MUL]++;
			ee_hist[ee * HIST_MUL]++;
		}
	}
}

double GetAccuracy(std::map< lld, lld > &hist, int percentile) {
	lld sval = pix_sum * percentile / 100;
	lld val = 0;
	for (auto ac : hist) { // ¡¡Puro AC!!
		val += ac.second;
		if (val >= sval) {
			return ac.first / HIST_MUL;
		}
	}
}

double GetSD(std::map< lld, lld > &hist, double mean) {
	double d, v = 0;
	for (auto ac : hist) {
		d = ac.first / HIST_MUL - mean;
		v +=  d * d * (double)ac.second;
	}
	return std::sqrt(v / (double)pix_sum);
}

void WriteFlo() {
	int width, height;
	FILE *stream = fopen("filename", "wb");
	fprintf(stream, TAG_STRING);
	fwrite(&width,  sizeof(int), 1, stream);
	fwrite(&height, sizeof(int), 1, stream);

	//code for flo file
	float xF = (float)1.0;
	float yF = (float)1.0;
	fwrite(&xF, sizeof(float), 1, stream);
	fwrite(&yF, sizeof(float), 1, stream);
}

void ReadFile(cv::Mat &gtx, cv::Mat &gty, std::string dir, int frame) {
	std::string fileName = dir + "_" + std::to_string(frame) + ".txt";
	FILE *in = fopen(fileName.c_str(), "r");
	double x, y;
	for (int i = 0; i < gtx.rows; i++){
		for (int j = 0; j < gtx.cols; j++){
			fscanf(in, "(%lf,%lf)", &x, &y);
			gtx.at<double>(i, j) = x;
			gty.at<double>(i, j) = y;
		}
		while(fgetc(in) != '\n');
	}
}

void PrintResults(std::string dir) {
	std::string fileName = dir + "_res.txt";
	FILE *out = fopen(fileName.c_str(), "w");
	double x, y;
	fprintf(out, "Angular\n");
	fprintf(out, "Avg = %lf\n", ae_sum / pix_sum);
	for (auto r : ae_r) {
		fprintf(out, "R%.3lf : %.2lf\n", r.first, ((double)r.second / (double)pix_sum) * 100.0);
	}
	fprintf(out, "Endpoint\n");
	fprintf(out, "Avg = %lf\n", ee_sum / pix_sum);
	for (auto r : ee_r) {
		fprintf(out, "R%.1lf : %.2lf\n", r.first, ((double)r.second / (double)pix_sum) * 100.0);
	}
	fclose(out);

	fileName = dir + "_hist.csv";
	out = fopen(fileName.c_str(), "w");
	lld i = 100000;
	auto p = ee_hist.begin();
	for (; p != ee_hist.end();  i += 100000) {
		lld sum = 0;
		while (p != ee_hist.end() && p->first < i) {
			sum += p->second;
			p++;
		}
		if (sum != 0LL)
			fprintf(out, "%lf,%lf\n", (double)i / HIST_MUL, (double)sum);
	}
}


//read nameFile starting_number 
//write to flow10.flo

int nomain(int argc, char** argv) {
	if (argc < 3) {
		int a;
		std::cout << argc << "Video input file and output directory required.";
		std::cin >> a;
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

	
	int width = 300;
	int height = 200;

	int orig_width;
	int orig_height;

	/*
	TestGenerator::GenerateTest("C:\\Users\\Adonais\\Desktop\\test.avi", 320, 240, 5);

	return 0;
	*/
	
	LucasKanade lk;

	VideoFactory lk_vf(dir + "-lk-flow.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

	VideoFactory ee_vf(dir + "-lk-ee.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

	cv::Mat vx, vy;
	std::cout << "\n\nStarting process.\n";
	int fps = (int)vcapture.get(CV_CAP_PROP_FPS);
	Frame* lk_result = new Frame(false);
	Frame* ee_result = new Frame(true);
	
	//int fileNumber = 7;
	//char* number = new char[3];

	std::vector< double > ae_rl;
	std::vector< double > ee_rl;

	ae_rl.push_back((CV_PI / 180.0) * 2.5);
	ae_rl.push_back((CV_PI / 180.0) * 5.0);
	ae_rl.push_back((CV_PI / 180.0) * 10.0);
	
	ee_rl.push_back(0.5);
	ee_rl.push_back(1.0);
	ee_rl.push_back(2.0);

	InitStats(ae_rl, ee_rl);
	
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
			ee_result->SetMatrix(&capture);
			ee_result->Rescale(width, height);
			ee_result->GetMatrixOnCache();
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

		cv::Mat mask = cv::Mat(height, width, CV_8U);
		cv::Mat gtx = cv::Mat(height, width, CV_64F);
		cv::Mat gty = cv::Mat(height, width, CV_64F);
		cv::Mat aem = cv::Mat(height, width, CV_64F);
		cv::Mat eem = cv::Mat(height, width, CV_64F);

		ReadFile(gtx, gty, dir, i);

		for (int i = 0; i < mask.rows; i++){
			for (int j = 0; j < mask.cols; j++){
				mask.at<uchar>(i, j) = 1;
			}
		}
		
		CalcStats(vx, vy, gtx, gty, mask, aem, eem);
		
		for (int x = 0; x < height; ++x) {
			double* ptr_vx = vx.ptr<double>(x);
			double* ptr_vy = vy.ptr<double>(x);
			for (int y = 0; y < width; ++y, ++ptr_vx, ++ptr_vy) {
				double X = *ptr_vx, Y = *ptr_vy;

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
		lk_result->GetCacheOnMatrix();
		lk_vf.AddFrame(lk_result->GetMatrix());

		for (int x = 0; x < height; ++x) {
			double* ptr_ee = eem.ptr<double>(x);
			for (int y = 0; y < width; ++y, ++ptr_ee) {
				ee_result->SetPixel(x, y, std::min(255, (int)(255 * (*ptr_ee / 10.0))));
			}
		}
		//fclose(stream);
		ee_result->GetCacheOnMatrix();
		ee_vf.AddFrame(ee_result->GetMatrix());
	}

	PrintResults(dir);
	
	
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
