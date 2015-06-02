#pragma once

#include <map>
#include <vector>
#include <ctime>

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
	std::vector< int > ae_ac, ee_ac;
	std::vector< std::clock_t > times;

	StatsTracker();
	double CalcAngularError(double u, double v, double gtu, double gtv);
	double CalcEndpointError(double u, double v, double gtu, double gtv);
	void CountMeasuresAboveThreshold(double err, RX &r);
	void InitStats(std::vector< double > ae_r_l, std::vector< double > ee_r_l, std::vector< int > ae_ac_l, std::vector< int > ee_ac_l);
	void CalcStats(cv::Mat &u, cv::Mat &v, cv::Mat &gtu, cv::Mat &gtv, cv::Mat &mask, cv::Mat &aem, cv::Mat &eem);
	double GetAccuracy(std::map< lld, lld > &hist, int percentile);
	double GetSD(std::map< lld, lld > &hist, double mean);
	void ReadFile(cv::Mat &gtx, cv::Mat &gty, std::string dir, int frame);
	void PrintResults(std::string dir);
	double GetAngularErrorAvg();
	double GetEndpointErrorAvg();
	std::map< double, lld > GetEndpointErrHistogram(double start, double finish, double step);
	std::map< double, lld > GetAngularErrHistogram(double start, double finish, double step);
	void AddTime(std::clock_t time);
	std::vector< std::clock_t > GetTimes();
	~StatsTracker();
private:
	std::map< double, lld > GetHistogram(std::map< lld, lld > e_hist, double start, double finish, double step);
};

