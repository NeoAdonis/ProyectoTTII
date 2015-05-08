#ifndef SIMPLEFLOW_H_
#define SIMPLEFLOW_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>
#include <limits>

#include "opencv2/imgproc/imgproc.hpp"

#include "frame.h"

class SimpleFlow{
public:
	// Frame control methods
	cv::Mat AddFrame(Frame*);
	void RemoveFrame();
	// Flow calculation method
	void CalculateFlow(cv::Mat& vel_x, cv::Mat& vel_y);

private:
	std::vector<Frame*> frames;
	void FillDistanceWeightMatrix();
	void BuildPyramid(Frame &src, std::vector<Frame>& pyramid);
	double GetEnergy(Frame &f1, int x1, int y1, Frame &f2, int x2, int y2);
	double GetWr(double *energyArray, int energySize);
	double GetWd(int x0, int y0, int x, int y);
	double GetWc(Frame &f1, int x0, int y0, int x, int y);
	//double GetWc(cv::Mat &f1, int x0, int y0, int x, int y);
	double getSmoothness(Frame &f1, Frame &f2, int x0, int y0, int x, int y);
	cv::Mat SimpleFlow::CheckIrreg(cv::Mat& irreg, int rows, int cols);
	cv::Mat SimpleFlow::UpscaleFlow(cv::Mat& flow, int new_rows, int new_cols, Frame &image, std::vector< std::vector<bool> >& isOccludedPixel);
	void CalcStageFlow(Frame& cur, Frame& next, cv::Mat& flow_x, cv::Mat& flow_y, cv::Mat& irreg);
	void BilateralFilter(cv::Mat flow_x, cv::Mat flow_y, Frame cur, Frame next, cv::Mat confidence, std::vector< std::vector<bool> >& isOccludedPixel);
	void CrossBilateralFilter(cv::Mat &orig, Frame &edge, std::vector< std::vector<bool> >& isOccludedPixel, cv::Mat& dest);
	void CalcOcclusion(cv::Mat& vel_x, cv::Mat& vel_y, cv::Mat& vel_x_inv, cv::Mat& vel_y_inv, std::vector< std::vector<bool> >& isOccludedPixel);
	void CalcConfidence(Frame& cur, Frame& next, cv::Mat& confidence);
	void CalcIrregularityMatrix(cv::Mat& flow_x, cv::Mat& flow_y, cv::Mat& irreg_mat);
	double** distanceWeight;
	void CalcRegularFlow(cv::Mat& flow_x, cv::Mat& flow_y, cv::Mat& irreg);
	static const int NeighborhoodSize;
	static const int Layers;
	static const double rd;
	static const double rc;
	static const double occlusion_limit;
	static const double threshold;
};


#endif