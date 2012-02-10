#include "ShiftBrite.h"
#include "libraries/SPI/SPI.h"

ShiftBrite::ShiftBrite(int dp, int lp, int ep, int cp) {
	m_mosi = dp;
	m_latch = lp;
	m_ss = ep;
	m_sck = cp;
	initPins();
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
}

void ShiftBrite::setPower(int r, int g, int b) {
	sendPacket(B01, r, g, b); //B01 sets the mode to write to the current registers
}

void ShiftBrite::setPower(int p) {
	sendPacket(B01, p, p, p);
}

void ShiftBrite::setColor(int r, int g, int b) {
	sendPacket(B00, constrain(r, 0, 1023), constrain(g, 0, 1023),
			constrain(b, 0, 1023)); //B00 sets the mode to write to the PWM registers
}

void ShiftBrite::initPins() {
	//	pinMode(m_mosi, OUTPUT);
	pinMode(m_latch, OUTPUT);
	pinMode(m_ss, OUTPUT);
	//	pinMode(m_sck, OUTPUT);
	digitalWrite(m_latch, LOW);
	digitalWrite(m_ss, HIGH);
}

void ShiftBrite::sendPacket(int mode, int r, int g, int b) {
	unsigned long SB_CommandPacket;

	SB_CommandPacket = mode & B11;
	SB_CommandPacket = (SB_CommandPacket << 10) | (b & 1023);
	SB_CommandPacket = (SB_CommandPacket << 10) | (r & 1023);
	SB_CommandPacket = (SB_CommandPacket << 10) | (g & 1023);

	digitalWrite(m_ss, LOW);
	SPI.transfer(SB_CommandPacket >> 24);
	SPI.transfer(SB_CommandPacket >> 16);
	SPI.transfer(SB_CommandPacket >> 8);
	SPI.transfer(SB_CommandPacket);
	latch();

}

void ShiftBrite::latch() {
	digitalWrite(m_latch, HIGH); // latch data into registers
	delayMicroseconds(1); // adjustment may be necessary depending on chain length
	digitalWrite(m_latch, LOW);
}
