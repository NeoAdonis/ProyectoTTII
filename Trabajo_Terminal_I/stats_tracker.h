#pragma once

#include <map>
#include <vector>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

typedef long long int lld;
typedef std::map< double, lld > RX;

class StatsTracker
{
public:
	lld pix_sum;
	double ae_sum, ee_sum;
	RX ae_r, ee_r;
	std::map< lld, lld > ae_hist, ee_hist;

	StatsTracker();
	double CalcAngularError(double u, double v, double gtu, double gtv);
	double CalcEndpointError(double u, double v, double gtu, double gtv);
	void CountMeasuresAboveThreshold(double err, RX &r);
	void InitStats(std::vector< double > ae_r_l, std::vector< double > ee_r_l);
	void CalcStats(cv::Mat &u, cv::Mat &v, cv::Mat &gtu, cv::Mat &gtv, cv::Mat &mask, cv::Mat &aem, cv::Mat &eem);
	double GetAccuracy(std::map< lld, lld > &hist, int percentile);
	double GetSD(std::map< lld, lld > &hist, double mean);
	void ReadFile(cv::Mat &gtx, cv::Mat &gty, std::string dir, int frame);
	void PrintResults(std::string dir);
	double GetAngularErrorAvg();
	double GetEndpointErrorAvg();
	~StatsTracker();
};

