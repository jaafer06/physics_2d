#include <iostream>
#include <string>
#include <physics.h>
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

class Example : public olc::PixelGameEngine {
public:
	Example()
	{
		sAppName = "Example";
	}

public:
	bool OnUserCreate() override
	{
		// Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		Clear(olc::BLACK);
		float mx = float(GetMouseX());
		float my = float(GetMouseY());
		Rect r = Rect{ 100, 100, 100, 100 };
		Rect r2 = Rect{ mx - 25, my - 25, 50, 50 };
		FillRect(mx-25, my-25, 50, 50, olc::BLUE);
		auto result = r.intersectionTest(r2);
		if (result.intersects) {
			FillRect(100, 100, 100, 100, olc::BLUE);
			float x1 = result.intersectionPoint[0];
			float y1 = result.intersectionPoint[1];
			float x2 = result.intersectionPoint[0] + result.normal[0]*50;
			float y2 = result.intersectionPoint[1] + result.normal[1]*50;
			DrawLine(x1, y1, x2, y2);
		} else {
			FillRect(100, 100, 100, 100, olc::RED);
		}
		
		return true;
	}
};


int main()
{
	Example demo;
	if (demo.Construct(1080, 780, 1, 1))
		demo.Start();

	return 0;
}

