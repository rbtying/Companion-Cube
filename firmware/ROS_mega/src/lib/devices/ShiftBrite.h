#ifndef ShiftBrite_h
#define ShiftBrite_h

#include "Arduino.h"

class ShiftBrite {
private:
	int m_mosi; //pins the ShiftBrite is connected to
	int m_latch;
	int m_ss;
	int m_sck;

	void initPins(); //sets up pins modes, etc.
	void sendPacket(int mode, int r, int g, int b);

public:
	ShiftBrite(int dp, int lp, int ep, int cp);
	void setPower(int r, int g, int b);
	void setPower(int p);
	void latch();
	void setColor(int r, int g, int b);
};

#endif
