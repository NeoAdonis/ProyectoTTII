#pragma once

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
			this->pbSource = (gcnew System::Windows::Forms::PictureBox());
			this->pbFlow = (gcnew System::Windows::Forms::PictureBox());
			this->lblEe = (gcnew System::Windows::Forms::Label());
			this->lblAe = (gcnew System::Windows::Forms::Label());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbSource))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbFlow))->BeginInit();
			this->SuspendLayout();
			// 
			// pbSource
			// 
			this->pbSource->Location = System::Drawing::Point(0, 0);
			this->pbSource->Name = L"pbSource";
			this->pbSource->Size = System::Drawing::Size(322, 143);
			this->pbSource->TabIndex = 0;
			this->pbSource->TabStop = false;
			// 
			// pbFlow
			// 
			this->pbFlow->Location = System::Drawing::Point(0, 159);
			this->pbFlow->Name = L"pbFlow";
			this->pbFlow->Size = System::Drawing::Size(322, 143);
			this->pbFlow->TabIndex = 1;
			this->pbFlow->TabStop = false;
			// 
			// lblEe
			// 
			this->lblEe->AutoSize = true;
			this->lblEe->Location = System::Drawing::Point(346, 13);
			this->lblEe->Name = L"lblEe";
			this->lblEe->Size = System::Drawing::Size(110, 13);
			this->lblEe->TabIndex = 2;
			this->lblEe->Text = L"Endpoint Error (avg.): ";
			// 
			// lblAe
			// 
			this->lblAe->AutoSize = true;
			this->lblAe->Location = System::Drawing::Point(383, 120);
			this->lblAe->Name = L"lblAe";
			this->lblAe->Size = System::Drawing::Size(110, 13);
			this->lblAe->TabIndex = 3;
			this->lblAe->Text = L"Angular Error (avg.): --";
			// 
			// TesterForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(760, 420);
			this->Controls->Add(this->lblAe);
			this->Controls->Add(this->lblEe);
			this->Controls->Add(this->pbFlow);
			this->Controls->Add(this->pbSource);
			this->Name = L"TesterForm";
			this->Text = L"TesterForm";
			this->Load += gcnew System::EventHandler(this, &TesterForm::TesterForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbSource))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pbFlow))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

		Threading::Thread ^t;
		static PictureBox ^pbSourceT, ^pbFlowT;
		static Label ^lblEeT, ^lblAeT;
		static std::string *dirT;
		static TesterForm ^curr;
		static StatsTracker *track;

		static void Convert(cv::Mat &img, PictureBox ^pb) {
			System::Drawing::Graphics^ graphics = pb->CreateGraphics();
			System::IntPtr ptr(img.ptr());
			System::Drawing::Bitmap^ b = gcnew System::Drawing::Bitmap(img.cols, img.rows, img.step.p[0], System::Drawing::Imaging::PixelFormat::Format24bppRgb, ptr);
			pb->Image = b;
		}

		static void InitRelErrParams(std::vector< double > &ae_rl, std::vector< double > &ee_rl) {
			ae_rl.push_back((CV_PI / 180.0) * 2.5);
			ae_rl.push_back((CV_PI / 180.0) * 5.0);
			ae_rl.push_back((CV_PI / 180.0) * 10.0);
			ee_rl.push_back(0.5);
			ee_rl.push_back(1.0);
			ee_rl.push_back(2.0);
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
			lblAeT->Text = gcnew System::String("Angular Error (avg.): ") + System::Convert::ToString(track->GetAngularErrorAvg() * 180.0 / CV_PI);
		}

		static void CalcFlowThread()
		{
			std::string dir = *dirT;

			cv::VideoCapture vcapture;
			vcapture.open(dir);
			if (!vcapture.isOpened()) {
				System::Windows::Forms::MessageBox::Show("Could not initialize capturing.", "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
				return;
			}

			cv::Mat capture;

			int width = 300, height = 200;
			int orig_width, orig_height;

			LucasKanade lk;

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
			InitRelErrParams(ae_rl, ee_rl);

			track->InitStats(ae_rl, ee_rl);

			for (int i = 0; i <= 100; ++i) {

				vcapture >> capture;
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
				frame->Rescale(width, height);
				frame->GetMatrixOnCache();
				lk.AddFrame(frame);
				lk.CalculateFlow(vx, vy);
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
			}
			track->PrintResults(dir);
		}

	private: System::Void TesterForm_Load(System::Object^  sender, System::EventArgs^  e) {
		curr = this;
		dirT = new std::string("H:\\Test\\test.avi");
		pbSourceT = pbSource;
		pbFlowT = pbFlow;
		lblAeT = lblAe;
		lblEeT = lblEe;
		t = gcnew Thread(gcnew ThreadStart(&this->CalcFlowThread));
		t->Start();
	}

};
}
