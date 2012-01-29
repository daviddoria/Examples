#include <iostream>
using namespace std;

#include <windows.h>
#include <gdiplus.h>

//#include <atlconv.h>
//#include <afxpriv.h>
//#include "atlbase.h"
//#include "Atlcom.h"

using namespace Gdiplus;


#pragma comment(lib,"gdiplus.lib")

int main()
{
	//GdiplusStartupInput gdiplusStartupInput;
	//ULONG_PTR gdiplusToken;

	int InputLeft = 100;
	int InputTop = 100;
	int OutputWidth = 300;
	int OutputHeight = 300;
	int OutputLeft = 1000;
	int OutputTop = 100;

	HDC Screen = GetDC(0);
	HBITMAP Capture = CreateCompatibleBitmap(Screen, OutputWidth, OutputHeight);
	HDC Image = CreateCompatibleDC(Screen);
	HBITMAP OldBitmap = (HBITMAP)SelectObject(Image, Capture);
	BitBlt(Image, OutputLeft, OutputTop, OutputWidth, OutputHeight, Screen, InputLeft, InputTop, SRCCOPY);
	SelectObject(Image, OldBitmap);

	Bitmap* bm = Bitmap::FromHBITMAP(Capture,NULL);
	
	CLSID png;
	Image::Save(L"C:/test.bmp", &png, NULL);
	

	int i;
	cin >> i;
	return 0;
}