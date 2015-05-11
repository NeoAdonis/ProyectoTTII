#include "stats_tracker.h"

#include <cmath>


StatsTracker::StatsTracker()
{
}


StatsTracker::~StatsTracker()
{
}

double StatsTracker::CalcAngularError(double u, double v, double gtu, double gtv) {
	return std::acos((1.0 + u * gtu + v * gtv) / (std::sqrt(1.0 + u * u + v * v) * std::sqrt(1.0 + gtu * gtu + gtv * gtv)));
}

double StatsTracker::CalcEndpointError(double u, double v, double gtu, double gtv) {
	double du = u - gtu, dv = v - gtv;
	return std::sqrt(du * du + dv * dv);
}

void StatsTracker::CountMeasuresAboveThreshold(double err, RX &r) {
	for (auto r_p = r.begin(); r_p != r.end(); r_p++) {
		if (err >= r_p->first)
			r_p->second++;
	}
}


const double HIST_MUL = 1e6;

void StatsTracker::InitStats(std::vector< double > ae_r_l, std::vector< double > ee_r_l) {
	pix_sum = 0;
	ae_sum = ee_sum = 0.0;
	for (double ae_r_v : ae_r_l) {
		ae_r[ae_r_v] = 0;
	}
	for (double ee_r_v : ee_r_l) {
		ee_r[ee_r_v] = 0;
	}
}

void StatsTracker::CalcStats(cv::Mat &u, cv::Mat &v, cv::Mat &gtu, cv::Mat &gtv, cv::Mat &mask, cv::Mat &aem, cv::Mat &eem) {
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

double StatsTracker::GetAccuracy(std::map< lld, lld > &hist, int percentile) {
	lld sval = pix_sum * percentile / 100;
	lld val = 0;
	for (auto ac : hist) { // ¡¡Puro AC!!
		val += ac.second;
		if (val >= sval) {
			return ac.first / HIST_MUL;
		}
	}
}

double StatsTracker::GetSD(std::map< lld, lld > &hist, double mean) {
	double d, v = 0;
	for (auto ac : hist) {
		d = ac.first / HIST_MUL - mean;
		v += d * d * (double)ac.second;
	}
	return std::sqrt(v / (double)pix_sum);
}

void StatsTracker::ReadFile(cv::Mat &gtx, cv::Mat &gty, std::string dir, int frame) {
	std::string fileName = dir + "_" + std::to_string(frame) + ".txt";
	FILE *in = fopen(fileName.c_str(), "r");
	double x, y;
	for (int i = 0; i < gtx.rows; i++){
		for (int j = 0; j < gtx.cols; j++){
			fscanf(in, "(%lf,%lf)", &x, &y);
			gtx.at<double>(i, j) = x;
			gty.at<double>(i, j) = y;
		}
		while (fgetc(in) != '\n');
	}
}

std::map< double, lld > StatsTracker::GetHistogram(std::map< lld, lld > e_hist, double sta, double fi, double st) {
	lld step = st * HIST_MUL;
	lld start = sta * HIST_MUL;
	lld finish = fi * HIST_MUL;
	lld i = start;
	auto p = e_hist.begin();
	std::map< double, lld > hist;
	for (; p != e_hist.end() && p->first < i; p++);
	for (i += step; p != e_hist.end(); i += step) {
		lld sum = 0;
		while (p != e_hist.end() && p->first < i) {
			sum += p->second;
			p++;
		}
		if (sum != 0LL)
			hist[(double)i / HIST_MUL] = sum;
		if (fi >= 0.0 && i > finish)
			break;
	}
	return hist;
}

std::map< double, lld > StatsTracker::GetEndpointErrHistogram(double start, double finish, double step) {
	return GetHistogram(ee_hist, start, finish, step);
}

std::map< double, lld > StatsTracker::GetAngularErrHistogram(double start, double finish, double step) {
	return GetHistogram(ae_hist, start, finish, step);
}

void StatsTracker::PrintResults(std::string dir) {
	std::string fileName = dir + "_res.txt";
	FILE *out = fopen(fileName.c_str(), "w");
	double x, y;
	fprintf(out, "Angular\n");
	fprintf(out, "Avg = %lf\n", GetAngularErrorAvg());
	for (auto r : ae_r) {
		fprintf(out, "R%.3lf : %.2lf\n", r.first, ((double)r.second / (double)pix_sum) * 100.0);
	}
	fprintf(out, "Endpoint\n");
	fprintf(out, "Avg = %lf\n", GetEndpointErrorAvg());
	for (auto r : ee_r) {
		fprintf(out, "R%.1lf : %.2lf\n", r.first, ((double)r.second / (double)pix_sum) * 100.0);
	}
	fclose(out);

	fileName = dir + "_hist.csv";
	out = fopen(fileName.c_str(), "w");
	auto hist = GetEndpointErrHistogram(0.0, -1, 0.1);
	for (auto p : hist) {
		fprintf(out, "%lf,%lf\n", p.first, (double)p.second);
	}
}

double StatsTracker::GetAngularErrorAvg() {
	return (double)ae_sum / (double)pix_sum;
}

double StatsTracker::GetEndpointErrorAvg() {
	return (double)ee_sum / (double)pix_sum;
}

void StatsTracker::AddTime(std::clock_t time) {
	times.push_back(time);
}

std::vector< std::clock_t > StatsTracker::GetTimes() {
	return times;
}