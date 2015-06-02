#pragma once

#include <ctime>
#include <string>
#include <map>

#include <msclr\marshal_cppstd.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "frame.h"
#include "lucas_kanade.h"
#include "video_factory.h"
#include "horn_schunck.h"
#include "simple_flow.h"
#include "stats_tracker.h"

#define TAG_STRING "PIEH"

const int kUp = 0x00FFFF; // LIGHT BLUE
const int kDown = 0x00FF00; // GREEN
const int kLeft = 0xFF0000; // RED
const int kRight = 0xFFFF00; // YELLOW
const double kIntensity = 4.7;


namespace Trabajo_Terminal_I {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Threading;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for TesterForm
	/// </summary>
	public ref class TesterForm : public System::Windows::Forms::Form
	{
	public:
		TesterForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~TesterForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::PictureBox^  pbSource;
	private: System::Windows::Forms::PictureBox^  pbFlow;
	private: System::Windows::Forms::Label^  lblEe;
	private: System::Windows::Forms::Label^  lblAe;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chartEe;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chartAe;
	private: System::Windows::Forms::Label^  lblTime;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chartTimes;

	protected:

	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea2 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea3 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->pbSource = (gcnew System::Windows::Forms::PictureBox());
			this->pbFlow = (gcnew System::Windows::Forms::PictureBox());
			this->lblEe = (gcnew System::Windows::Forms::Label());
			this->lblAe = (gcnew System::Windows::Forms::Label());
			this->chartEe = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->chartAe = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->lblTime = (gcnew System::Windows::Forms::Label());
			this->chartTimes = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbFlow))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartEe))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartAe))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartTimes))->BeginInit();
			this->SuspendLayout();
			// 
			// pbSource
			// 
			this->pbSource->Location = System::Drawing::Point(0, 0);
			this->pbSource->Name = L"pbSource";
			this->pbSource->Size = System::Drawing::Size(300, 200);
			this->pbSource->TabIndex = 0;
			this->pbSource->TabStop = false;
			// 
			// pbFlow
			// 
			this->pbFlow->Location = System::Drawing::Point(0, 206);
			this->pbFlow->Name = L"pbFlow";
			this->pbFlow->Size = System::Drawing::Size(300, 200);
			this->pbFlow->TabIndex = 1;
			this->pbFlow->TabStop = false;
			// 
			// lblEe
			// 
			this->lblEe->AutoSize = true;
			this->lblEe->Location = System::Drawing::Point(306, 9);
			this->lblEe->Name = L"lblEe";
			this->lblEe->Size = System::Drawing::Size(116, 13);
			this->lblEe->TabIndex = 2;
			this->lblEe->Text = L"Endpoint Error (avg.): --";
			// 
			// lblAe
			// 
			this->lblAe->AutoSize = true;
			this->lblAe->Location = System::Drawing::Point(306, 32);
			this->lblAe->Name = L"lblAe";
			this->lblAe->Size = System::Drawing::Size(110, 13);
			this->lblAe->TabIndex = 3;
			this->lblAe->Text = L"Angular Error (avg.): --";
			// 
			// chartEe
			// 
			chartArea1->Name = L"ChartArea1";
			this->chartEe->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->chartEe->Legends->Add(legend1);
			this->chartEe->Location = System::Drawing::Point(309, 58);
			this->chartEe->Name = L"chartEe";
			series1->ChartArea = L"ChartArea1";
			series1->Legend = L"Legend1";
			series1->Name = L"Endpoint Err";
			this->chartEe->Series->Add(series1);
			this->chartEe->Size = System::Drawing::Size(347, 171);
			this->chartEe->TabIndex = 4;
			this->chartEe->Text = L"chart1";
			// 
			// chartAe
			// 
			chartArea2->Name = L"ChartArea1";
			this->chartAe->ChartAreas->Add(chartArea2);
			legend2->Name = L"Legend1";
			this->chartAe->Legends->Add(legend2);
			this->chartAe->Location = System::Drawing::Point(309, 237);
			this->chartAe->Name = L"chartAe";
			series2->ChartArea = L"ChartArea1";
			series2->Legend = L"Legend1";
			series2->Name = L"Angular Err";
			this->chartAe->Series->Add(series2);
			this->chartAe->Size = System::Drawing::Size(347, 171);
			this->chartAe->TabIndex = 5;
			this->chartAe->Text = L"chart1";
			// 
			// lblTime
			// 
			this->lblTime->AutoSize = true;
			this->lblTime->Location = System::Drawing::Point(659, 9);
			this->lblTime->Name = L"lblTime";
			this->lblTime->Size = System::Drawing::Size(42, 13);
			this->lblTime->TabIndex = 6;
			this->lblTime->Text = L"Time: --";
			// 
			// chartTimes
			// 
			chartArea3->Name = L"ChartArea1";
			this->chartTimes->ChartAreas->Add(chartArea3);
			legend3->Name = L"Legend1";
			this->chartTimes->Legends->Add(legend3);
			this->chartTimes->Location = System::Drawing::Point(662, 58);
			this->chartTimes->Name = L"chartTimes";
			series3->ChartArea = L"ChartArea1";
			series3->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Spline;
			series3->Legend = L"Legend1";
			series3->Name = L"Time";
			this->chartTimes->Series->Add(series3);
			this->chartTimes->Size = System::Drawing::Size(347, 171);
			this->chartTimes->TabIndex = 7;
			this->chartTimes->Text = L"chart1";
			// 
			// TesterForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1019, 416);
			this->Controls->Add(this->chartTimes);
			this->Controls->Add(this->lblTime);
			this->Controls->Add(this->chartAe);
			this->Controls->Add(this->chartEe);
			this->Controls->Add(this->lblAe);
			this->Controls->Add(this->lblEe);
			this->Controls->Add(this->pbFlow);
			this->Controls->Add(this->pbSource);
			this->Name = L"TesterForm";
			this->Text = L"Tester";
			this->Load += gcnew System::EventHandler(this, &TesterForm::TesterForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbSource))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbFlow))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartEe))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartAe))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chartTimes))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

		Threading::Thread ^t;
		static PictureBox ^pbSourceT, ^pbFlowT;
		static Label ^lblEeT, ^lblAeT, ^lblTimeT;
		static System::Windows::Forms::DataVisualization::Charting::Chart ^chartEeT, ^chartAeT, ^chartTimesT;
		static std::string *dirT;
		static TesterForm ^curr;
		static StatsTracker *track;
		static std::clock_t *time;
		static bool doMask;

		static void Convert(cv::Mat &img, PictureBox ^pb) {
			System::Drawing::Graphics^ graphics = pb->CreateGraphics();
			System::IntPtr ptr(img.ptr());
			System::Drawing::Bitmap^ b = gcnew System::Drawing::Bitmap(img.cols, img.rows, img.step.p[0], System::Drawing::Imaging::PixelFormat::Format24bppRgb, ptr);
			pb->Image = b;
		}

		static void InitRelErrParams(std::vector< double > &ae_rl, std::vector< double > &ee_rl) {
			ae_rl.push_back(2.5);
			ae_rl.push_back(5.0);
			ae_rl.push_back(10.0);
			ee_rl.push_back(0.5);
			ee_rl.push_back(1.0);
			ee_rl.push_back(2.0);
		}

		static void InitAcParams(std::vector< int > &ae_ac, std::vector< int > &ee_ac) {
			ae_ac.push_back(50);
			ae_ac.push_back(75);
			ae_ac.push_back(95);
			ee_ac.push_back(90);
			ee_ac.push_back(95);
			ee_ac.push_back(99);
		}

		static void InitFrame(cv::Mat &capture, Frame *frame, int width, int height) {
			frame->SetMatrix(&capture);
			frame->Rescale(width, height);
			frame->GetMatrixOnCache();
		}

		static void EditEELabel() {
			lblEeT->Text = gcnew System::String("Endpoint Error (avg.): ") + System::Convert::ToString(track->GetEndpointErrorAvg());
		}

		static void EditAELabel() {
			lblAeT->Text = gcnew System::String("Angular Error (avg.): ") + System::Convert::ToString(track->GetAngularErrorAvg()) + gcnew System::String("°");
		}

		static void EditTimeLabel() {
			lblTimeT->Text = gcnew System::String("Time: ") + System::Convert::ToString(*time) + gcnew System::String(" ms");
		}

		static void EditEEChart() {
			for (auto p : track->GetEndpointErrHistogram(0.1, 10.0, 0.1)) {
				chartEeT->Series["Endpoint Err"]->Points->AddXY(p.first, p.second);
			}
		}

		static void EditAEChart() {
			for (auto p : track->GetAngularErrHistogram(0.1, 10.0, 0.1)) {
				chartAeT->Series["Angular Err"]->Points->AddXY(p.first, p.second);
			}
		}

		static void EditTimesChart() {
			int i = 0;
			chartTimesT->Series["Time"]->Points->Clear();
			for (auto p : track->GetTimes()) {
				chartTimesT->Series["Time"]->Points->AddXY(i, p);
				i++;
			}
		}


		static void CloseApp() {
			curr->Close();
		}

		static void CalcFlowThread()
		{
			std::string dir = *dirT;

			cv::VideoCapture vcapture, mcapture;
			vcapture.open(dir);
			if (!vcapture.isOpened()) {
				System::Windows::Forms::MessageBox::Show("Could not initialize capturing.", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
				curr->Invoke(gcnew Action(&CloseApp));
				return;
			}
			if (doMask) {
				mcapture.open(dir + "_border.avi");
				if (!mcapture.isOpened()) {
					System::Windows::Forms::MessageBox::Show("Could not find border mask.", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
					curr->Invoke(gcnew Action(&CloseApp));
					return;
				}
			}

			cv::Mat capture, cmask;

			int width = 300, height = 200;
			int orig_width, orig_height;

			LucasKanade flow;

			VideoFactory lk_vf(dir + "-lk-flow.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));
			VideoFactory ee_vf(dir + "-lk-ee.avi", width, height, vcapture.get(CV_CAP_PROP_FPS));

			cv::Mat vx, vy;
			
			int fps = (int)vcapture.get(CV_CAP_PROP_FPS);

			Frame* lk_result = new Frame(false);
			Frame* ee_result = new Frame(true);

			track = new StatsTracker();
			cv::Mat mask = cv::Mat(height, width, CV_8U);
			cv::Mat gtx = cv::Mat(height, width, CV_64F);
			cv::Mat gty = cv::Mat(height, width, CV_64F);
			cv::Mat aem = cv::Mat(height, width, CV_64F);
			cv::Mat eem = cv::Mat(height, width, CV_64F);
			for (int i = 0; i < mask.rows; i++){
				for (int j = 0; j < mask.cols; j++){
					mask.at<uchar>(i, j) = 1;
				}
			}
			std::vector< double > ae_rl;
			std::vector< double > ee_rl;
			std::vector< int > ae_ac;
			std::vector< int > ee_ac;
			InitRelErrParams(ae_rl, ee_rl);
			InitAcParams(ae_ac, ee_ac);

			track->InitStats(ae_rl, ee_rl, ae_ac, ee_ac);

			for (int i = 0; i < 30; ++i) {

				vcapture >> capture;
				if (doMask) {
					mcapture >> cmask;
					for (int i = 0; i < mask.rows; i++){
						for (int j = 0; j < mask.cols; j++){
							mask.at<uchar>(i, j) = cmask.at<uchar>(i, j) ? 1 : 0;
						}
					}
				}
				if (capture.empty()) break;

				if (i == 0) {
					InitFrame(capture, lk_result, width, height);
					InitFrame(capture, ee_result, width, height);
				}

				Frame* origFrame = new Frame(&capture, false);
				orig_width = origFrame->Columns();
				orig_height = origFrame->Rows();
				origFrame->Rescale(width, height);
				origFrame->GetMatrixOnCache();
				cv::Mat img = origFrame->GetMatrix();
				if (origFrame->IsGrayscale()) {
					cv::cvtColor(img, img, CV_GRAY2RGB);
				}
				else {
					cv::cvtColor(img, img, CV_BGR2RGB);
				}
				Convert(img, pbSourceT);

				Frame* frame = new Frame(&capture);
				std::clock_t start_time = std::clock();
				frame->Rescale(width, height);
				frame->GetMatrixOnCache();
				flow.AddFrame(frame);
				flow.CalculateFlow(vx, vy);
				std::clock_t ptime = (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000);
				*time = ptime;
				track->AddTime(ptime);
				lk_result->AddFlow(vx, vy);
				img = lk_result->GetMatrix();
				cv::cvtColor(img, img, CV_BGR2RGB);
				Convert(img, pbFlowT);
				lk_result->GetCacheOnMatrix();
				lk_vf.AddFrame(lk_result->GetMatrix());
				track->ReadFile(gtx, gty, dir, i);
				track->CalcStats(vx, vy, gtx, gty, mask, aem, eem);
				curr->Invoke(gcnew Action(&EditAELabel));
				curr->Invoke(gcnew Action(&EditEELabel));
				curr->Invoke(gcnew Action(&EditTimeLabel));
				curr->Invoke(gcnew Action(&EditTimesChart));
			}
			curr->Invoke(gcnew Action(&EditAEChart));
			curr->Invoke(gcnew Action(&EditEEChart));
			track->PrintResults(dir);
		}

	private: System::Void TesterForm_Load(System::Object^  sender, System::EventArgs^  e) {
		std::string dir;
		curr = this;
		OpenFileDialog ^ofd = gcnew OpenFileDialog();
		if (ofd->ShowDialog() == System::Windows::Forms::DialogResult::OK) {
			dir = msclr::interop::marshal_as<std::string>(ofd->FileName);
			dirT = new std::string(dir);
		}
		else {
			Close();
		}
		doMask = false;
		pbSourceT = pbSource;
		pbFlowT = pbFlow;
		lblAeT = lblAe;
		lblEeT = lblEe;
		lblTimeT = lblTime;
		chartEeT = chartEe;
		chartAeT = chartAe;
		chartTimesT = chartTimes;
		time = new std::clock_t;
		t = gcnew Thread(gcnew ThreadStart(&this->CalcFlowThread));
		t->Start();
		this->Focus();
	}

};
}
