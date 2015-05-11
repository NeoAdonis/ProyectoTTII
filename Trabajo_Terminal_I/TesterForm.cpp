#include "TesterForm.h"
#include <Windows.h>

#using <mscorlib.dll>
#using <System.dll>
#using <System.Windows.Forms.dll>

using namespace System::Windows::Forms;

[System::STAThread]
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd) {
	Application::Run(gcnew Trabajo_Terminal_I::TesterForm);
}