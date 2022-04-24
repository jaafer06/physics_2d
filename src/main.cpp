#include <iostream>
#include <string>
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include <physics.h>
constexpr float pi = 3.14159265358979323846;


class Example : public olc::PixelGameEngine {
public:
	void draw(const Rect& rect) {
		const auto [p0, p1, p2, p3] = rect.getPoints();
		DrawLine(p0[0], -p0[1], p1[0], -p1[1], olc::BLUE);
		DrawLine(p1[0], -p1[1], p2[0], -p2[1], olc::BLUE);
		DrawLine(p2[0], -p2[1], p3[0], -p3[1], olc::BLUE);
		DrawLine(p3[0], -p3[1], p0[0], -p0[1], olc::BLUE);
	}

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
		float my = -float(GetMouseY());
		Rect r = Rect{ mx, my, 50, 50 };
		Rect r2 = Rect{ 100, -100, 100, 100 };


		draw(r);
		draw(r2);
		r2.hm(r);
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

