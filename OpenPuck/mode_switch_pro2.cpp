// mode_switch_pro2.cpp -- Nintendo Switch 2 Pro Controller (057E:2069).
//
// The USB descriptors are byte-for-byte captures of a physical controller.
// Native 0x09 reports carry the 30-byte projective-quaternion + Q16.16
// accelerometer motion block used by the Switch 2 console, while 0x05 reports
// carry raw 6-axis IMU samples used by SDL3/Steam.
#include "mode_switch_pro2.h"
#include "bonds.h"
#include "config.h"
#include "gamepad_util.h"
#include "haptics.h"
#include "rf_link.h"
#include "triton.h"
#include "usb_app_drivers.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "class/hid/hid_device.h"
#include "device/usbd_pvt.h"
}

SwitchPro2Controller g_switchPro2;

static const uint8_t SWITCH2_PRO_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x05, 0x05, 0xff, 0x09,
	0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x95, 0x3f, 0x75, 0x08, 0x81,
	0x02, 0x85, 0x09, 0x09, 0x01, 0x95, 0x02, 0x81, 0x02, 0x05, 0x09,
	0x19, 0x01, 0x29, 0x15, 0x25, 0x01, 0x95, 0x15, 0x75, 0x01, 0x81,
	0x02, 0x95, 0x01, 0x75, 0x03, 0x81, 0x03, 0x05, 0x01, 0x09, 0x01,
	0xa1, 0x00, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x35, 0x26,
	0xff, 0x0f, 0x95, 0x04, 0x75, 0x0c, 0x81, 0x02, 0xc0, 0x05, 0xff,
	0x09, 0x02, 0x26, 0xff, 0x00, 0x95, 0x34, 0x75, 0x08, 0x91, 0x02,
	0x85, 0x02, 0x09, 0x01, 0x95, 0x3f, 0x91, 0x02, 0xc0,
};
static_assert(sizeof SWITCH2_PRO_HID_DESC == 97,
	      "Switch 2 Pro HID descriptor must remain byte-exact");

// Everything after the 9-byte configuration header. Interface and endpoint
// numbers are fixed because this clean personality is the only mounted USB
// function: IF0 HID, IF1 vendor bulk, IF2-4 Nintendo audio shape.
static const uint8_t SWITCH2_PRO_CFG_BODY[] = {
	0x08, 0x0b, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x09, 0x04, 0x00, 0x00,
	0x02, 0x03, 0x00, 0x00, 0x05, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
	0x61, 0x00, 0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x04, 0x07, 0x05, 0x01,
	0x03, 0x40, 0x00, 0x04, 0x08, 0x0b, 0x01, 0x01, 0xff, 0x00, 0x00, 0x00,
	0x09, 0x04, 0x01, 0x00, 0x02, 0xff, 0x00, 0x00, 0x06, 0x07, 0x05, 0x02,
	0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x08,
	0x0b, 0x02, 0x03, 0x01, 0x01, 0x00, 0x00, 0x09, 0x04, 0x02, 0x00, 0x00,
	0x01, 0x01, 0x00, 0x00, 0x0a, 0x24, 0x01, 0x00, 0x01, 0x47, 0x00, 0x02,
	0x03, 0x04, 0x0c, 0x24, 0x02, 0x01, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00,
	0x00, 0x00, 0x0a, 0x24, 0x06, 0x02, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00,
	0x09, 0x24, 0x03, 0x03, 0x02, 0x03, 0x00, 0x02, 0x00, 0x0c, 0x24, 0x02,
	0x04, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x09, 0x24, 0x06,
	0x05, 0x04, 0x01, 0x03, 0x00, 0x00, 0x09, 0x24, 0x03, 0x06, 0x01, 0x01,
	0x00, 0x05, 0x00, 0x09, 0x04, 0x03, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
	0x09, 0x04, 0x03, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x07, 0x24, 0x01,
	0x01, 0x00, 0x01, 0x00, 0x0b, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10, 0x01,
	0x80, 0xbb, 0x00, 0x07, 0x05, 0x03, 0x0d, 0xc0, 0x00, 0x01, 0x07, 0x25,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x09, 0x04, 0x04, 0x00, 0x00, 0x01, 0x02,
	0x00, 0x00, 0x09, 0x04, 0x04, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x07,
	0x24, 0x01, 0x06, 0x00, 0x01, 0x00, 0x0b, 0x24, 0x02, 0x01, 0x02, 0x02,
	0x10, 0x01, 0x80, 0xbb, 0x00, 0x07, 0x05, 0x83, 0x0d, 0xc0, 0x00, 0x01,
	0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,
};
static_assert(sizeof SWITCH2_PRO_CFG_BODY == 259,
	      "Switch 2 Pro configuration body must remain byte-exact");

static volatile uint8_t g_sw2ActiveReport = 0x09;
static volatile uint8_t g_sw2Features = 0;
static volatile uint8_t g_sw2FeatureMask = 0;
static volatile bool g_sw2InputEnabled = false;
static volatile uint8_t g_sw2Counter8 = 0;
static volatile uint32_t g_sw2Counter32 = 0;
static uint32_t g_sw2TimestampUs = 0;
static uint16_t g_sw2SampleCounter = 0;
static uint8_t g_sw2ImuReg0A = 0x76;
static float g_sw2Quat[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static int8_t g_sw2LastRumbleBond = -1;
static uint16_t g_sw2LastRumbleLeft = 0;
static uint16_t g_sw2LastRumbleRight = 0;
static bool g_sw2RumbleActive = false;
static unsigned long g_sw2SampleEndMs = 0;
static volatile bool g_sw2VendorCommandPending = false;
static volatile bool g_sw2VendorInFlight = false;
static volatile bool g_sw2VendorOutRearmPending = false;
static volatile uint8_t g_sw2VendorCommandLen = 0;
static volatile uint8_t g_sw2VendorReplyLen = 0;
static volatile uint8_t g_sw2VendorReplyOffset = 0;
static uint8_t g_sw2VendorOut[64];
static uint8_t g_sw2VendorCmd[64];
static uint8_t g_sw2VendorReply[96];
static uint8_t g_sw2ControlReply[64];
static const uint8_t SW2_VENDOR_IDENTITY[64] = {
	0x01, 0x00, 'H',  'E',	'W',  '7',  '0',  '0',	'0',  '6',  '1',
	'6',  '9',  '7',  '8',	'0',  0x00, 0x00, 0x7e, 0x05, 0x69, 0x20,
	0x01, 0x06, 0x01, 0x23, 0x23, 0x23, 0xa0, 0xa0, 0xa0, 0xe6, 0xe6,
	0xe6, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static_assert(sizeof SW2_VENDOR_IDENTITY == 64,
	      "Switch 2 Pro vendor identity must remain 64 bytes");

static const uint8_t SW2_VENDOR_PROTOCOL[16] = {
	0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0c, 0x00,
	0x02, 0x03, 0x3d, 0x17, 0x69, 0xab, 0xa9, 0x3c,
};

static volatile uint8_t g_sw2VendorEpOut = 0;
static volatile uint8_t g_sw2VendorEpIn = 0;
static volatile uint8_t g_sw2Rhport = 0;
static unsigned long g_sw2LastReportMs = 0;
static bool g_sw2DrainRegistered = false;

class Switch2ProUsbInterface : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buf,
					uint16_t bufsize) override
	{
		if (!buf)
			return sizeof SWITCH2_PRO_CFG_BODY;
		if (bufsize < sizeof SWITCH2_PRO_CFG_BODY)
			return 0;

		uint8_t first = TinyUSBDevice.allocInterface(5);
		uint8_t hidIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		uint8_t hidOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t vendorOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t vendorIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		uint8_t audioOut = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t audioIn = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		if (first != 0 || hidIn != 0x81 || hidOut != 0x01 ||
		    vendorOut != 0x02 || vendorIn != 0x82 || audioOut != 0x03 ||
		    audioIn != 0x83)
			return 0;

		uint8_t configStr =
			TinyUSBDevice.addStringDescriptor("Config_0");
		uint8_t hidStr = TinyUSBDevice.addStringDescriptor("If_Hid");
		uint8_t vendorStr = TinyUSBDevice.addStringDescriptor(
			"Switch 2 Pro Controller");
		if (configStr != 4 || hidStr != 5 || vendorStr != 6)
			return 0;

		memcpy(buf, SWITCH2_PRO_CFG_BODY, sizeof SWITCH2_PRO_CFG_BODY);
		return sizeof SWITCH2_PRO_CFG_BODY;
	}
};
static Switch2ProUsbInterface g_sw2Usb;

static inline uint16_t sw2Stick12(int16_t v)
{
	return (uint16_t)(((int32_t)v + 32768) >> 4);
}

static inline void sw2PackStick(uint8_t out[3], int16_t x, int16_t y)
{
	uint16_t sx = sw2Stick12(x), sy = sw2Stick12(y);
	out[0] = (uint8_t)sx;
	out[1] = (uint8_t)((sx >> 8) | (sy << 4));
	out[2] = (uint8_t)(sy >> 4);
}

static void sw2SetCode(uint8_t code, uint8_t b09[3], uint8_t b05[4])
{
	switch (code) {
	case 1: // A
		b09[0] |= 0x02;
		b05[0] |= 0x08;
		break;
	case 2: // B
		b09[0] |= 0x01;
		b05[0] |= 0x04;
		break;
	case 3: // X
		b09[0] |= 0x08;
		b05[0] |= 0x02;
		break;
	case 4: // Y
		b09[0] |= 0x04;
		b05[0] |= 0x01;
		break;
	case 5: // L
		b09[1] |= 0x10;
		b05[2] |= 0x40;
		break;
	case 6: // R
		b09[0] |= 0x10;
		b05[0] |= 0x40;
		break;
	case 7: // left stick
		b09[1] |= 0x80;
		b05[1] |= 0x08;
		break;
	case 8: // right stick
		b09[0] |= 0x80;
		b05[1] |= 0x04;
		break;
	case 9: // minus
		b09[1] |= 0x40;
		b05[1] |= 0x01;
		break;
	case 10: // plus
		b09[0] |= 0x40;
		b05[1] |= 0x02;
		break;
	case 11: // home
		b09[2] |= 0x01;
		b05[1] |= 0x10;
		break;
	case 12: // up
		b09[1] |= 0x08;
		b05[2] |= 0x02;
		break;
	case 13: // down
		b09[1] |= 0x01;
		b05[2] |= 0x01;
		break;
	case 14: // left
		b09[1] |= 0x04;
		b05[2] |= 0x08;
		break;
	case 15: // right
		b09[1] |= 0x02;
		b05[2] |= 0x04;
		break;
	case 18: // capture
		b09[2] |= 0x02;
		b05[1] |= 0x20;
		break;
	case 19: // ZL
		b09[1] |= 0x20;
		b05[2] |= 0x80;
		break;
	case 20: // ZR
		b09[0] |= 0x20;
		b05[0] |= 0x80;
		break;
	case 21: // GL
		b09[2] |= 0x08;
		b05[3] |= 0x02;
		break;
	case 22: // GR
		b09[2] |= 0x04;
		b05[3] |= 0x01;
		break;
	case 23: // C
		b09[2] |= 0x10;
		b05[1] |= 0x40;
		break;
	default:
		break;
	}
}

static void sw2Buttons(uint8_t slot, uint8_t b09[3], uint8_t b05[4])
{
	memset(b09, 0, 3);
	memset(b05, 0, 4);
	uint32_t b = g_in[slot].buttons | padDpadButtons(g_in[slot]);
	bool qam = g_qamMap && (b & TB_QAM);
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP | TB_DDN |
				 TB_DLF | TB_DRT);
	if (b & TB_A)
		sw2SetCode(g_abSwap ? 2 : 1, b09, b05);
	if (b & TB_B)
		sw2SetCode(g_abSwap ? 1 : 2, b09, b05);
	if (b & TB_X)
		sw2SetCode(g_abSwap ? 4 : 3, b09, b05);
	if (b & TB_Y)
		sw2SetCode(g_abSwap ? 3 : 4, b09, b05);
	if (b & TB_LB)
		sw2SetCode(5, b09, b05);
	if (b & TB_RB)
		sw2SetCode(6, b09, b05);
	if (g_in[slot].lt >= SW_TRIG_ON || (b & 0x8000000u))
		sw2SetCode(19, b09, b05);
	if (g_in[slot].rt >= SW_TRIG_ON || (b & 0x800000u))
		sw2SetCode(20, b09, b05);
	if (b & TB_L3)
		sw2SetCode(7, b09, b05);
	if (b & TB_R3)
		sw2SetCode(8, b09, b05);
	if (b & TB_VIEW)
		sw2SetCode(10, b09, b05);
	if (b & TB_MENU)
		sw2SetCode(9, b09, b05);
	if (b & TB_STEAM)
		sw2SetCode(11, b09, b05);
	if (b & TB_DUP)
		sw2SetCode(12, b09, b05);
	if (b & TB_DDN)
		sw2SetCode(13, b09, b05);
	if (b & TB_DLF)
		sw2SetCode(14, b09, b05);
	if (b & TB_DRT)
		sw2SetCode(15, b09, b05);
	if (b & TB_L4)
		sw2SetCode(g_back[0], b09, b05);
	if (b & TB_R4)
		sw2SetCode(g_back[1], b09, b05);
	if (b & TB_L5)
		sw2SetCode(g_back[2], b09, b05);
	if (b & TB_R5)
		sw2SetCode(g_back[3], b09, b05);
	if (qam)
		sw2SetCode(g_qamMap, b09, b05);
	else if (b & TB_QAM)
		sw2SetCode(23, b09, b05);
}

static uint8_t sw2PowerInfo(uint8_t slot)
{
	uint8_t pct = (slot < NSLOT && g_battery[slot]) ? g_battery[slot] : 100;
	uint8_t level = (uint8_t)(((uint16_t)pct * 9u + 50u) / 100u);
	if (level > 9)
		level = 9;
	uint8_t chg = (slot < NSLOT && g_batteryState[slot] == 2) ? 0x02 : 0x00;
	return (uint8_t)((level << 2) | chg);
}

static void sw2ImuAxes(uint8_t slot, int16_t *axOut, int16_t *ayOut,
		       int16_t *azOut, int16_t *gxOut, int16_t *gyOut,
		       int16_t *gzOut)
{
	int16_t ax = (int16_t)(g_in[slot].ax / 4);
	int16_t ay = (int16_t)(g_in[slot].ay / 4);
	int16_t az = (int16_t)(g_in[slot].az / 4);
	int16_t gx = (int16_t)g_in[slot].gx;
	int16_t gy = (int16_t)g_in[slot].gy;
	int16_t gz = (int16_t)g_in[slot].gz;
	if (!g_swGyroLegacy) {
		gx = (int16_t)(((int32_t)gx * 9) / 10);
		gy = (int16_t)(((int32_t)gy * 4) / 5);
		gz = (int16_t)(((int32_t)gz * 9) / 10);
	}
	*axOut = ax;
	*ayOut = ay;
	*azOut = az;
	*gxOut = gx;
	*gyOut = gy;
	*gzOut = gz;
}

// Report 0x09's 30-byte motion payload encodes 4 integrated 960 Hz gyro samples
// as a projective unit quaternion (pivot index in m[4], three Q1.24 cyclic
// ratios in m[5..15]) followed by Q16.16 body-frame accelerometer axes in
// m[16..27] and temperature in m[28..29].
static void sw2PackMotion09(uint8_t slot, uint8_t m[30])
{
	int16_t ax, ay, az, gx, gy, gz;
	sw2ImuAxes(slot, &ax, &ay, &az, &gx, &gy, &gz);

	const uint8_t nsamp = 4;
	g_sw2SampleCounter = (uint16_t)((g_sw2SampleCounter + nsamp) & 0x0fffu);
	m[0] = (uint8_t)(g_sw2SampleCounter & 0xffu);
	m[1] = (uint8_t)((nsamp << 4) | ((g_sw2SampleCounter >> 8) & 0x0fu));
	m[2] = 0x00;
	m[3] = 0x0c;

	// Half-angle step per LSB for 4 samples at 960 Hz and +-2000 dps full
	// scale: 0.5 * (4 / 960) * ((2000 * pi / 180) / 32768).
	const float step = 2.2193009e-6f;
	float hx = (float)gx * step;
	float hy = (float)gy * step;
	float hz = (float)gz * step;
	float q0 = g_sw2Quat[0], q1 = g_sw2Quat[1];
	float q2 = g_sw2Quat[2], q3 = g_sw2Quat[3];
	float n0 = q0 - q1 * hx - q2 * hy - q3 * hz;
	float n1 = q1 + q0 * hx + q2 * hz - q3 * hy;
	float n2 = q2 + q0 * hy - q1 * hz + q3 * hx;
	float n3 = q3 + q0 * hz + q1 * hy - q2 * hx;
	float norm = sqrtf(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
	if (norm > 1e-6f) {
		float inv = 1.0f / norm;
		g_sw2Quat[0] = n0 * inv;
		g_sw2Quat[1] = n1 * inv;
		g_sw2Quat[2] = n2 * inv;
		g_sw2Quat[3] = n3 * inv;
	} else {
		g_sw2Quat[0] = 1.0f;
		g_sw2Quat[1] = 0.0f;
		g_sw2Quat[2] = 0.0f;
		g_sw2Quat[3] = 0.0f;
	}

	uint8_t idx = 0;
	float maxAbs = fabsf(g_sw2Quat[0]);
	for (uint8_t i = 1; i < 4; i++) {
		float a = fabsf(g_sw2Quat[i]);
		if (a > maxAbs) {
			maxAbs = a;
			idx = i;
		}
	}
	m[4] = idx;
	float denom = g_sw2Quat[idx];
	uint32_t u[3];
	for (uint8_t i = 0; i < 3; i++) {
		float r = g_sw2Quat[(idx + 1u + i) & 3u] / denom;
		int32_t g = (int32_t)(r * 16777216.0f);
		if (g < -16777216)
			g = -16777216;
		else if (g > 16777215)
			g = 16777215;
		u[i] = (uint32_t)(g + 0x01000000);
	}
	uint32_t w0 = u[0] << 1;
	uint32_t w1 = (u[1] & 0x03ffffffu) | ((u[2] & 1u) << 31);
	uint32_t w2 = u[2] >> 1;
	memcpy(m + 5, &w0, 4);
	memcpy(m + 9, &w1, 4);
	m[13] = (uint8_t)w2;
	m[14] = (uint8_t)(w2 >> 8);
	m[15] = (uint8_t)(w2 >> 16);

	int32_t a0 = (int32_t)ax << 16;
	int32_t a1 = (int32_t)ay << 16;
	int32_t a2 = (int32_t)az << 16;
	memcpy(m + 16, &a0, 4);
	memcpy(m + 20, &a1, 4);
	memcpy(m + 24, &a2, 4);
	m[28] = 0x00;
	m[29] = 0x00;
}

static void sw2Build09(uint8_t slot, uint8_t out[63])
{
	memset(out, 0, 63);
	uint8_t b09[3], b05[4];
	sw2Buttons(slot, b09, b05);
	out[0] = g_sw2Counter8++;
	out[1] = sw2PowerInfo(slot);
	memcpy(out + 2, b09, sizeof b09);
	int16_t lx, ly, rx, ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 5, lx, ly);
	sw2PackStick(out + 8, rx, ry);
	out[0x0b] = (g_sw2Features & (1u << 5)) ? 0x38 : 0x30;
	if (g_sw2Features & 0x04) {
		out[0x0e] = 0x1e;
		sw2PackMotion09(slot, out + 0x0f);
	}
}

static inline void sw2Put16(uint8_t *p, int16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)((uint16_t)v >> 8);
}

static void sw2Build05(uint8_t slot, uint8_t out[63])
{
	memset(out, 0, 63);
	uint8_t b09[3], b05[4];
	sw2Buttons(slot, b09, b05);
	uint32_t counter = g_sw2Counter32++;
	memcpy(out, &counter, sizeof counter);
	memcpy(out + 4, b05, sizeof b05);
	int16_t lx, ly, rx, ry;
	slotSticks(slot, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 0x0a, lx, ly);
	sw2PackStick(out + 0x0d, rx, ry);
	out[0x1f] = 0xd8;
	out[0x20] = 0x0e;
	out[0x21] = 0x34;
	out[0x29] = 0x01;

	uint8_t *m = out + 0x2a;
	g_sw2TimestampUs += 4000u;
	memcpy(m, &g_sw2TimestampUs, sizeof g_sw2TimestampUs);
	int16_t ax, ay, az, gx, gy, gz;
	sw2ImuAxes(slot, &ax, &ay, &az, &gx, &gy, &gz);
	// m[4..5] temperature stays zero; sensor axes begin at m[6].
	sw2Put16(m + 6, ax);
	sw2Put16(m + 8, ay);
	sw2Put16(m + 10, az);
	sw2Put16(m + 12, gx);
	sw2Put16(m + 14, gy);
	sw2Put16(m + 16, gz);
}

static void sw2AckHeader(uint8_t *out, uint8_t cmd, uint8_t transport,
			 uint8_t sub)
{
	out[0] = cmd;
	out[1] = 0x01;
	out[2] = transport;
	out[3] = sub;
	out[4] = 0x00;
	out[5] = 0xf8;
	out[6] = 0x00;
	out[7] = 0x00;
}

static void sw2DataHeader(uint8_t *out, uint8_t cmd, uint8_t transport,
			  uint8_t sub)
{
	out[0] = cmd;
	out[1] = 0x01;
	out[2] = transport;
	out[3] = sub;
	out[4] = transport == 0x01 ? 0x10 : 0x00;
	out[5] = transport == 0x01 ? 0x78 : 0xf8;
	out[6] = 0x00;
	out[7] = 0x00;
}

static uint8_t sw2QueueDataHeader(uint8_t cmd, uint8_t transport, uint8_t sub,
				  uint8_t *reply)
{
	sw2DataHeader(reply, cmd, transport, sub);
	return 8;
}

static const uint8_t SW2_FACTORY_13040[16] = {
	0x3b, 0xe0, 0xd3, 0x41, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t SW2_FACTORY_13080[49] = {
	0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a,
	0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a,
	0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5,
	0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff,
	0xb3, 0x67, 0x83, 0x2e, 0x66, 0x5e, 0x3a, 0x06, 0x5f,
};

static const uint8_t SW2_FACTORY_130C0[49] = {
	0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a,
	0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a,
	0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5,
	0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff,
	0x2c, 0x08, 0x84, 0xd1, 0x65, 0x63, 0x2a, 0x26, 0x62,
};

// Bytes 0..11 are zero gyro biases; bytes 12..23 are resting accelerometer
// gravity (0.0f, 0.0f, +9.80665f m/s^2).
static const uint8_t SW2_FACTORY_13100[24] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x41,
};

static const uint8_t SW2_FACTORY_13140[9] = {
	0x00, 0xd7, 0xa3, 0xbc, 0x41, 0xd7, 0xa3, 0xbc, 0x41,
};

static const uint8_t SW2_FLASH_175000[26] = {
	0x44, 0x53, 0x50, 0x48, 0x01, 0x00, 0x00, 0x00, 0xa0,
	0x96, 0x02, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00,
	0x09, 0x10, 0xfd, 0x16, 0x29, 0x8a, 0x06, 0x5f,
};

static uint8_t g_sw2UserCal[256];
static bool g_sw2UserCalInit = false;

static void sw2EnsureUserCal(void)
{
	if (!g_sw2UserCalInit) {
		memset(g_sw2UserCal, 0xff, sizeof g_sw2UserCal);
		g_sw2UserCalInit = true;
	}
}

static void sw2OverlayFlash(uint32_t address, uint8_t *out, uint8_t len,
			    uint32_t knownAddress, const uint8_t *known,
			    uint16_t knownLen)
{
	uint32_t begin = address > knownAddress ? address : knownAddress;
	uint32_t end = address + len;
	uint32_t knownEnd = knownAddress + knownLen;
	if (end > knownEnd)
		end = knownEnd;
	if (begin >= end)
		return;
	memcpy(out + (begin - address), known + (begin - knownAddress),
	       end - begin);
}

static void sw2FillFlashBlock(uint32_t address, uint8_t *block, uint8_t len)
{
	sw2EnsureUserCal();
	// Uninitialised controller flash is 0xff; overlay captured Pro factory data.
	memset(block, 0xff, len);
	sw2OverlayFlash(address, block, len, 0x00013000, SW2_VENDOR_IDENTITY,
			sizeof SW2_VENDOR_IDENTITY);
	sw2OverlayFlash(address, block, len, 0x00013040, SW2_FACTORY_13040,
			sizeof SW2_FACTORY_13040);
	sw2OverlayFlash(address, block, len, 0x00013080, SW2_FACTORY_13080,
			sizeof SW2_FACTORY_13080);
	sw2OverlayFlash(address, block, len, 0x000130c0, SW2_FACTORY_130C0,
			sizeof SW2_FACTORY_130C0);
	sw2OverlayFlash(address, block, len, 0x00013100, SW2_FACTORY_13100,
			sizeof SW2_FACTORY_13100);
	sw2OverlayFlash(address, block, len, 0x00013140, SW2_FACTORY_13140,
			sizeof SW2_FACTORY_13140);
	sw2OverlayFlash(address, block, len, 0x00175000, SW2_FLASH_175000,
			sizeof SW2_FLASH_175000);
	sw2OverlayFlash(address, block, len, 0x001fc000, g_sw2UserCal,
			sizeof g_sw2UserCal);
}

static uint8_t sw2HandleFlashCommand(const uint8_t *cmd, uint8_t n,
				     uint8_t *reply)
{
	uint8_t sub = cmd[3];
	if (n < 16)
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);

	uint32_t address = (uint32_t)cmd[12] | ((uint32_t)cmd[13] << 8) |
			   ((uint32_t)cmd[14] << 16) |
			   ((uint32_t)cmd[15] << 24);
	if (sub == 0x03) {
		sw2EnsureUserCal();
		if (address >= 0x001fc000 && address < 0x001fd000)
			memset(g_sw2UserCal, 0xff, sizeof g_sw2UserCal);
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);
	}
	if (sub == 0x02 || sub == 0x05) {
		sw2EnsureUserCal();
		uint8_t writeLen = sub == 0x02 ? 0x40 : cmd[8];
		if (writeLen > n - 16)
			writeLen = (uint8_t)(n - 16);
		for (uint8_t i = 0; i < writeLen; i++) {
			uint32_t a = address + i;
			if (a >= 0x001fc000 && a < 0x001fc100)
				g_sw2UserCal[a - 0x001fc000] = cmd[16 + i];
		}
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);
	}

	uint8_t requested;
	if (sub == 0x01)
		requested = 0x40;
	else if (sub == 0x04)
		requested = cmd[8];
	else
		return sw2QueueDataHeader(0x02, cmd[2], sub, reply);

	sw2DataHeader(reply, 0x02, cmd[2], sub);
	uint8_t copied = requested > 80 ? 80 : requested;
	reply[8] = copied;
	reply[12] = cmd[12];
	reply[13] = cmd[13];
	reply[14] = cmd[14];
	reply[15] = cmd[15];
	sw2FillFlashBlock(address, reply + 16, copied);
	return (uint8_t)(16 + copied);
}

static void sw2FeatureInfo(uint8_t flags, uint8_t out[8])
{
	memset(out, 0, 8);
	if (flags & 0x01)
		out[0] = 0x07;
	if (flags & 0x02)
		out[1] = 0x07;
	if (flags & 0x04)
		out[2] = 0x01;
	if (flags & 0x80)
		out[3] = 0x01;
	if (flags & 0x10)
		out[4] = 0x01;
	if (flags & 0x20)
		out[5] = 0x03;
}

static uint8_t sw2HandleUsbCommand(const uint8_t *cmd, uint8_t n,
				   uint8_t *reply)
{
	uint8_t sub = cmd[3];
	sw2AckHeader(reply, 0x03, cmd[2], sub);
	if (sub == 0x03) {
		g_sw2InputEnabled = n >= 9 && cmd[8] != 0;
		reply[8] = 0x01;
		return 12;
	}
	if (sub == 0x0d) {
		g_sw2InputEnabled = true;
		reply[8] = 0x01;
		return 12;
	}
	if (sub == 0x0f) {
		reply[8] = 0x05;
		return 12;
	}
	if (sub == 0x07 || sub == 0x09) {
		sw2DataHeader(reply, 0x03, cmd[2], sub);
		return 8;
	}
	if (sub == 0x0a && n >= 9 && (cmd[8] == 0x05 || cmd[8] == 0x09))
		g_sw2ActiveReport = cmd[8];
	return 8;
}

static uint8_t g_sw2PairLtk[16];
static bool g_sw2PairLtkValid = false;
static const uint8_t SW2_PAIR_PUBLIC[16] = {
	0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
	0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
};

struct Sw2EcbBlock {
	uint8_t key[16];
	uint8_t clear[16];
	uint8_t cipher[16];
} __attribute__((aligned(4)));

static bool sw2Aes128Ecb(const uint8_t key[16], const uint8_t clear[16],
			 uint8_t cipher[16])
{
	static Sw2EcbBlock block;
	memcpy(block.key, key, 16);
	memcpy(block.clear, clear, 16);
	NRF_ECB->TASKS_STOPECB = 1;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	NRF_ECB->ECBDATAPTR = (uint32_t)&block;
	NRF_ECB->TASKS_STARTECB = 1;
	while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB) {
	}
	if (NRF_ECB->EVENTS_ERRORECB)
		return false;
	memcpy(cipher, block.cipher, 16);
	return true;
}

static void sw2PairResponse(const uint8_t challenge[16], uint8_t out[16])
{
	uint8_t key[16], clear[16], cipher[16];
	for (uint8_t i = 0; i < 16; i++) {
		key[i] = g_sw2PairLtk[15 - i];
		clear[i] = challenge[15 - i];
	}
	if (!g_sw2PairLtkValid || !sw2Aes128Ecb(key, clear, cipher)) {
		memset(out, 0, 16);
		return;
	}
	// Nintendo's confirmation packet uses the AES output in the order returned
	// by the hardware after reversing the key and challenge inputs.
	memcpy(out, cipher, 16);
}

static void sw2ControllerAddress(uint8_t out[6])
{
	uint32_t a = NRF_FICR->DEVICEID[0], b = NRF_FICR->DEVICEID[1];
	out[0] = (uint8_t)a;
	out[1] = (uint8_t)(a >> 8);
	out[2] = (uint8_t)(a >> 16);
	out[3] = (uint8_t)(a >> 24);
	out[4] = (uint8_t)b;
	out[5] = (uint8_t)(b >> 8);
}

static uint8_t sw2HandlePairing(const uint8_t *cmd, uint8_t n, uint8_t *reply)
{
	uint8_t sub = cmd[3];
	sw2DataHeader(reply, 0x15, cmd[2], sub);
	if (sub == 0x01) {
		uint8_t addr[6];
		sw2ControllerAddress(addr);
		reply[8] = 0x01;
		reply[9] = 0x04;
		reply[10] = 0x01;
		memcpy(reply + 11, addr, sizeof addr);
		return 17;
	}
	if (sub == 0x04 && n >= 25) {
		reply[8] = 0x01;
		memcpy(reply + 9, SW2_PAIR_PUBLIC, sizeof SW2_PAIR_PUBLIC);
		for (uint8_t i = 0; i < 16; i++)
			g_sw2PairLtk[i] = cmd[9 + i] ^ SW2_PAIR_PUBLIC[i];
		g_sw2PairLtkValid = true;
		return 25;
	}
	if (sub == 0x02 && n >= 25) {
		reply[8] = 0x01;
		sw2PairResponse(cmd + 9, reply + 9);
		return 25;
	}
	if (sub == 0x03) {
		reply[8] = 0x01;
		return 9;
	}
	reply[8] = 0x01;
	return 9;
}

static void sw2StopRumble(void)
{
	if (!g_sw2RumbleActive && g_sw2LastRumbleBond < 0)
		return;
	int bond = g_sw2LastRumbleBond >= 0 ?
			   g_sw2LastRumbleBond :
			   (g_usbMountCount ? g_usbToBond[0] : -1);
	g_sw2RumbleActive = false;
	g_sw2LastRumbleBond = -1;
	g_sw2LastRumbleLeft = 0;
	g_sw2LastRumbleRight = 0;
	if (bond >= 0 && bond < NSLOT)
		hapticSteamRumble(0, 0, (uint8_t)bond);
}

static uint8_t sw2HandleFeatures(const uint8_t *cmd, uint8_t n, uint8_t *reply)
{
	uint8_t sub = cmd[3], flags = n >= 9 ? cmd[8] : 0;
	sw2DataHeader(reply, 0x0c, cmd[2], sub);
	if (sub == 0x01) {
		sw2FeatureInfo(flags, reply + 12);
		return 20;
	}
	if (sub == 0x02)
		g_sw2FeatureMask = flags;
	else if (sub == 0x03) {
		g_sw2FeatureMask = 0;
		g_sw2Features = 0;
		sw2StopRumble();
	} else if (sub == 0x04) {
		if ((flags & 0x04) && !(g_sw2Features & 0x04)) {
			g_sw2SampleCounter = 0;
			g_sw2Quat[0] = 1.0f;
			g_sw2Quat[1] = 0.0f;
			g_sw2Quat[2] = 0.0f;
			g_sw2Quat[3] = 0.0f;
		}
		g_sw2Features |= flags & g_sw2FeatureMask;
	} else if (sub == 0x05) {
		g_sw2Features &= (uint8_t) ~(flags & g_sw2FeatureMask);
		if (flags & 0x20)
			sw2StopRumble();
	} else if (sub == 0x06) {
		reply[12] = n >= 13 ? cmd[12] : 0x02;
		if (n >= 18 && cmd[16] == 0x0a)
			g_sw2ImuReg0A = cmd[17];
		else if (n >= 17 && cmd[16] == 0x8a)
			reply[17] = g_sw2ImuReg0A;
		return 48;
	}
	return 12;
}

static void sw2PlayVibrationSample(uint8_t sampleId)
{
	// Switch 2 sends a burst of 4-6 sampleId=0 commands on every rumble
	// menu toggle; only enqueue RF stop packets when a sample or rumble is
	// actually active.
	if (sampleId == 0) {
		if (g_sw2SampleEndMs &&
		    (int32_t)(millis() - g_sw2SampleEndMs) < 0 &&
		    g_usbMountCount) {
			int bond = g_usbToBond[0];
			if (bond >= 0 && bond < NSLOT) {
				hapticAudioTone(0, -128, 160, 30,
						(uint8_t)bond);
				hapticAudioTone(1, -128, 160, 30,
						(uint8_t)bond);
			}
		}
		g_sw2SampleEndMs = 0;
		sw2StopRumble();
		return;
	}
	if (!g_usbMountCount || !g_rumble)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0 || bond >= NSLOT)
		return;
	uint16_t hz = (sampleId == 1) ? 160 : 320;
	uint16_t ms = (sampleId == 1 || sampleId == 2) ? 240 : 80;
	hapticAudioTone(0, -9, hz, ms, (uint8_t)bond);
	hapticAudioTone(1, -9, hz, ms, (uint8_t)bond);
	g_sw2SampleEndMs = millis() + ms;
	if (!g_sw2SampleEndMs)
		g_sw2SampleEndMs = 1;
}

static void sw2BuildVendorReply(void)
{
	uint8_t cmd[64];
	uint8_t reply[96];
	uint8_t n;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	n = g_sw2VendorCommandLen;
	if (n > sizeof cmd)
		n = sizeof cmd;
	memcpy(cmd, g_sw2VendorCmd, n);
	g_sw2VendorCommandPending = false;
	__set_PRIMASK(pm);

	memset(reply, 0, sizeof reply);
	if (n < 8 || cmd[1] != 0x91) {
		g_sw2VendorReplyOffset = 0;
		g_sw2VendorReplyLen = 0;
		return;
	}
	uint8_t id = cmd[0], sub = cmd[3], seq = cmd[2];
	sw2AckHeader(reply, id, seq, sub);
	uint8_t replyLen = 8;

	switch (id) {
	case 0x02:
		replyLen = sw2HandleFlashCommand(cmd, n, reply);
		break;
	case 0x03:
		replyLen = sw2HandleUsbCommand(cmd, n, reply);
		break;
	case 0x0c:
		replyLen = sw2HandleFeatures(cmd, n, reply);
		break;
	case 0x15:
		replyLen = sw2HandlePairing(cmd, n, reply);
		break;
	case 0x07:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		reply[8] = 0;
		replyLen = 9;
		break;
	case 0x01:
		if (sub == 0x0c) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = 0x61;
			reply[9] = 0x12;
			reply[10] = 0x50;
			reply[11] = 0x0d;
			replyLen = 12;
		}
		break;
	case 0x10:
		if (sub == 0x01) {
			static const uint8_t info[12] = {
				0x02, 0x01, 0x04, 0x02, 0x0c, 0x00,
				0x00, 0x00, 0x00, 0x02, 0x03, 0x00,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, info, sizeof info);
			replyLen = 20;
		}
		break;
	case 0x11:
		if (sub == 0x03) {
			static const uint8_t imuCal[] = {
				0x01, 0xc0, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c,
				0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c,
				0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c,
				0x41, 0x58, 0xa0, 0x0b, 0x41,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, imuCal, sizeof imuCal);
			replyLen = (uint8_t)(8 + sizeof imuCal);
		} else if (sub == 0x01) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = 0x03;
			replyLen = 12;
		}
		break;
	case 0x16:
		if (sub == 0x01) {
			sw2DataHeader(reply, id, seq, sub);
			memset(reply + 8, 0, 24);
			replyLen = 32;
		}
		break;
	case 0x18:
		if (sub == 0x01) {
			static const uint8_t powerInfo[8] = {
				0x00, 0x00, 0x40, 0xf0, 0x00, 0x00, 0x60, 0x00,
			};
			sw2DataHeader(reply, id, seq, sub);
			memcpy(reply + 8, powerInfo, sizeof powerInfo);
			replyLen = 16;
		} else if (sub == 0x03 && n >= 9) {
			sw2DataHeader(reply, id, seq, sub);
			reply[8] = cmd[8];
			replyLen = 9;
		}
		break;
	case 0x06:
		if (sub == 0x02) {
			g_sw2VendorReplyOffset = 0;
			g_sw2VendorReplyLen = 0;
			return;
		}
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	case 0x0a:
		if (sub == 0x02 && n >= 9)
			sw2PlayVibrationSample(cmd[8]);
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	case 0x09:
	case 0x0d:
	case 0x17:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	case 0x0b:
		sw2DataHeader(reply, id, seq, sub);
		if (sub == 0x03) {
			reply[8] = 0xa5;
			reply[9] = 0x0e;
			replyLen = 12;
		} else if (sub == 0x04) {
			reply[8] = 0x34;
			reply[10] = 0x83;
			replyLen = 12;
		} else if (sub == 0x06) {
			reply[8] = 0x11;
			replyLen = 12;
		}
		break;
	default:
		replyLen = sw2QueueDataHeader(id, seq, sub, reply);
		break;
	}

	if (replyLen > sizeof g_sw2VendorReply)
		replyLen = sizeof g_sw2VendorReply;
	memcpy(g_sw2VendorReply, reply, replyLen);
	g_sw2VendorReplyOffset = 0;
	g_sw2VendorReplyLen = replyLen;
}

static void sw2Drain(void)
{
	if (g_usbMode != MODE_SW_PRO2 || g_usbMountCount == 0)
		return;

	if (g_sw2VendorOutRearmPending && g_sw2VendorEpOut &&
	    !usbd_edpt_busy(g_sw2Rhport, g_sw2VendorEpOut) &&
	    usbd_edpt_claim(g_sw2Rhport, g_sw2VendorEpOut)) {
		if (usbd_edpt_xfer(g_sw2Rhport, g_sw2VendorEpOut,
				   g_sw2VendorOut, sizeof g_sw2VendorOut))
			g_sw2VendorOutRearmPending = false;
		else
			usbd_edpt_release(g_sw2Rhport, g_sw2VendorEpOut);
	}

	if (g_sw2VendorCommandPending && !g_sw2VendorInFlight &&
	    g_sw2VendorReplyLen == 0) {
		sw2BuildVendorReply();
		if (g_sw2VendorReplyLen == 0)
			g_sw2VendorOutRearmPending = true;
	}

	if (!g_sw2VendorInFlight &&
	    g_sw2VendorReplyOffset < g_sw2VendorReplyLen && g_sw2VendorEpIn &&
	    !usbd_edpt_busy(g_sw2Rhport, g_sw2VendorEpIn) &&
	    usbd_edpt_claim(g_sw2Rhport, g_sw2VendorEpIn)) {
		uint8_t rem =
			(uint8_t)(g_sw2VendorReplyLen - g_sw2VendorReplyOffset);
		uint8_t chunk = rem > 64 ? 64 : rem;
		uint8_t off = g_sw2VendorReplyOffset;
		if (usbd_edpt_xfer(g_sw2Rhport, g_sw2VendorEpIn,
				   g_sw2VendorReply + off, chunk)) {
			g_sw2VendorReplyOffset = (uint8_t)(off + chunk);
			g_sw2VendorInFlight = true;
			return;
		}
		usbd_edpt_release(g_sw2Rhport, g_sw2VendorEpIn);
	}

	if (!g_sw2InputEnabled || !tud_hid_n_ready(0))
		return;
	if ((uint32_t)(millis() - g_sw2LastReportMs) < USB_STREAM_MS)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0 || bond >= NSLOT)
		return;
	uint8_t p[63];
	uint8_t rid = g_sw2ActiveReport;
	if (rid == 0x05)
		sw2Build05((uint8_t)bond, p);
	else {
		rid = 0x09;
		sw2Build09((uint8_t)bond, p);
	}
	if (tud_hid_n_report(0, rid, p, sizeof p))
		g_sw2LastReportMs = millis();
}

bool switch2ProVendorControlXfer(uint8_t rhport, uint8_t stage,
				 const tusb_control_request_t *request)
{
	if (g_usbMode != MODE_SW_PRO2 || !request)
		return false;

	bool identity = request->bmRequestType == 0xc0 &&
			request->bRequest == 0x03 && request->wLength >= 64;
	bool protocol = request->bmRequestType == 0xc0 &&
			request->bRequest == 0x02 && request->wLength >= 16;
	bool commit = request->bmRequestType == 0x40 &&
		      request->bRequest == 0x04 && request->wLength == 0;
	if (!identity && !protocol && !commit)
		return false;
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (commit)
		return tud_control_status(rhport, request);
	if (identity) {
		memcpy(g_sw2ControlReply, SW2_VENDOR_IDENTITY,
		       sizeof SW2_VENDOR_IDENTITY);
		return tud_control_xfer(rhport, request, g_sw2ControlReply,
					sizeof SW2_VENDOR_IDENTITY);
	}
	memcpy(g_sw2ControlReply, SW2_VENDOR_PROTOCOL,
	       sizeof SW2_VENDOR_PROTOCOL);
	return tud_control_xfer(rhport, request, g_sw2ControlReply,
				sizeof SW2_VENDOR_PROTOCOL);
}

static void sw2DriverInit(void)
{
	g_sw2VendorEpOut = g_sw2VendorEpIn = 0;
}
static bool sw2DriverDeinit(void)
{
	return true;
}
static void sw2DriverReset(uint8_t rhport)
{
	(void)rhport;
	g_sw2VendorEpOut = g_sw2VendorEpIn = 0;
	g_sw2VendorCommandPending = false;
	g_sw2VendorInFlight = false;
	g_sw2VendorOutRearmPending = false;
	g_sw2VendorReplyLen = 0;
	g_sw2VendorReplyOffset = 0;
	g_sw2InputEnabled = false;
	g_sw2ActiveReport = 0x09;
	g_sw2Features = 0;
	g_sw2FeatureMask = 0;
	g_sw2SampleCounter = 0;
	g_sw2ImuReg0A = 0x76;
	g_sw2Quat[0] = 1.0f;
	g_sw2Quat[1] = 0.0f;
	g_sw2Quat[2] = 0.0f;
	g_sw2Quat[3] = 0.0f;
	g_sw2LastRumbleBond = -1;
	g_sw2LastRumbleLeft = 0;
	g_sw2LastRumbleRight = 0;
	g_sw2RumbleActive = false;
	g_sw2SampleEndMs = 0;
}

static uint16_t sw2DriverOpen(uint8_t rhport, tusb_desc_interface_t const *itf,
			      uint16_t maxLen)
{
	if (g_usbMode != MODE_SW_PRO2)
		return 0;
	g_sw2Rhport = rhport;
	if (itf->bInterfaceNumber == 0 &&
	    itf->bInterfaceClass == TUSB_CLASS_HID)
		return hidd_open(rhport, itf, maxLen);

	if (itf->bInterfaceNumber == 1 && itf->bInterfaceClass == 0xff) {
		uint8_t const *p = (uint8_t const *)itf;
		uint8_t const *end = p + maxLen;
		uint16_t used = 0;
		uint8_t opened = 0;
		while (p < end) {
			uint8_t len = p[0], type = p[1];
			if (!len)
				return 0;
			if (p != (uint8_t const *)itf &&
			    (type == TUSB_DESC_INTERFACE ||
			     type == TUSB_DESC_INTERFACE_ASSOCIATION))
				break;
			if (type == TUSB_DESC_ENDPOINT) {
				auto ep = (tusb_desc_endpoint_t const *)p;
				if (!usbd_edpt_open(rhport, ep))
					return 0;
				if (tu_edpt_dir(ep->bEndpointAddress) ==
				    TUSB_DIR_IN)
					g_sw2VendorEpIn = ep->bEndpointAddress;
				else
					g_sw2VendorEpOut = ep->bEndpointAddress;
				opened++;
			}
			used += len;
			p += len;
		}
		if (opened != 2 || !g_sw2VendorEpOut || !g_sw2VendorEpIn)
			return 0;
		if (!usbd_edpt_xfer(rhport, g_sw2VendorEpOut, g_sw2VendorOut,
				    sizeof g_sw2VendorOut))
			return 0;
		return used;
	}

	// Preserve the captured audio-control/streaming descriptor block as one
	// associated three-interface Nintendo function. Audio data itself is not
	// synthesized by OpenPuck.
	if (itf->bInterfaceNumber == 2 &&
	    itf->bInterfaceClass == TUSB_CLASS_AUDIO)
		return maxLen;
	return 0;
}

static bool sw2DriverControl(uint8_t rhport, uint8_t stage,
			     tusb_control_request_t const *req)
{
	if (g_usbMode != MODE_SW_PRO2)
		return false;
	if (req->wIndex == 0)
		return hidd_control_xfer_cb(rhport, stage, req);
	return stage != CONTROL_STAGE_SETUP;
}

static bool sw2DriverXfer(uint8_t rhport, uint8_t ep, xfer_result_t result,
			  uint32_t transferred)
{
	if (ep == g_sw2VendorEpOut) {
		if (result == XFER_RESULT_SUCCESS && transferred > 0) {
			uint8_t n =
				(uint8_t)(transferred > 64 ? 64 : transferred);
			memcpy(g_sw2VendorCmd, g_sw2VendorOut, n);
			g_sw2VendorCommandLen = n;
			g_sw2VendorCommandPending = true;
			return true;
		}
		if (!usbd_edpt_xfer(rhport, g_sw2VendorEpOut, g_sw2VendorOut,
				    sizeof g_sw2VendorOut))
			g_sw2VendorOutRearmPending = true;
		return true;
	}
	if (ep == g_sw2VendorEpIn) {
		if (result == XFER_RESULT_SUCCESS &&
		    g_sw2VendorReplyOffset < g_sw2VendorReplyLen) {
			uint8_t rem = (uint8_t)(g_sw2VendorReplyLen -
						g_sw2VendorReplyOffset);
			uint8_t chunk = rem > 64 ? 64 : rem;
			uint8_t off = g_sw2VendorReplyOffset;
			if (usbd_edpt_xfer(rhport, g_sw2VendorEpIn,
					   g_sw2VendorReply + off, chunk)) {
				g_sw2VendorReplyOffset = (uint8_t)(off + chunk);
				return true;
			}
			g_sw2VendorInFlight = false;
			return true;
		}
		g_sw2VendorReplyLen = 0;
		g_sw2VendorReplyOffset = 0;
		g_sw2VendorInFlight = false;
		if (!usbd_edpt_xfer(rhport, g_sw2VendorEpOut, g_sw2VendorOut,
				    sizeof g_sw2VendorOut))
			g_sw2VendorOutRearmPending = true;
		return true;
	}
	return hidd_xfer_cb(rhport, ep, result, transferred);
}

static const usbd_class_driver_t g_sw2Driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "SW2-PRO",
#endif
	.init = sw2DriverInit,
	.deinit = sw2DriverDeinit,
	.reset = sw2DriverReset,
	.open = sw2DriverOpen,
	.control_xfer_cb = sw2DriverControl,
	.xfer_cb = sw2DriverXfer,
	.sof = nullptr,
};

const usbd_class_driver_t *switch2ProClassDriver(void)
{
	return &g_sw2Driver;
}

extern "C" uint8_t const *__real_tud_hid_descriptor_report_cb(uint8_t itf);
extern "C" uint8_t const *__wrap_tud_hid_descriptor_report_cb(uint8_t itf)
{
	if (g_usbMode == MODE_SW_PRO2 && itf == 0)
		return SWITCH2_PRO_HID_DESC;
	return __real_tud_hid_descriptor_report_cb(itf);
}

extern "C" uint16_t __real_tud_hid_get_report_cb(uint8_t itf, uint8_t reportId,
						 hid_report_type_t reportType,
						 uint8_t *buffer,
						 uint16_t reqLen);
extern "C" uint16_t __wrap_tud_hid_get_report_cb(uint8_t itf, uint8_t reportId,
						 hid_report_type_t reportType,
						 uint8_t *buffer,
						 uint16_t reqLen)
{
	if (g_usbMode != MODE_SW_PRO2 || itf != 0)
		return __real_tud_hid_get_report_cb(itf, reportId, reportType,
						    buffer, reqLen);
	(void)reportType;
	if (!buffer || !reqLen)
		return 0;
	uint8_t p[63];
	int bond = g_usbMountCount ? g_usbToBond[0] : -1;
	if (bond < 0)
		memset(p, 0, sizeof p);
	else if (reportId == 0x05)
		sw2Build05((uint8_t)bond, p);
	else if (reportId == 0x09)
		sw2Build09((uint8_t)bond, p);
	else
		return 0;
	uint16_t n = reqLen < sizeof p ? reqLen : sizeof p;
	memcpy(buffer, p, n);
	return n;
}

// Switch 2 Pro output report 0x02 carries two 16-byte LRA packets after the
// report ID: left then right. Each packet starts with a 0x50|sequence byte and
// contains three absolute 5-byte HD-rumble frames. Sub-1% fade-out codes
// (< 576) are gated to silence so fade-out dithering does not keep the RF
// relay active after a rumble pulse ends.
static void sw2RumbleMotorBands(const uint8_t packet[16], uint16_t *lowOut,
				uint16_t *highOut)
{
	if ((packet[0] & 0xf0) != 0x50) {
		*lowOut = 0;
		*highOut = 0;
		return;
	}
	uint16_t lowPeak = 0, highPeak = 0;
	for (uint8_t i = 0; i < 3; i++) {
		const uint8_t *frame = packet + 1 + 5 * i;
		uint16_t highAmp =
			(uint16_t)(((uint16_t)(frame[1] & 0xfc) << 4) |
				   ((uint16_t)(frame[2] & 0x0f) << 12));
		uint16_t lowAmp = (uint16_t)((frame[3] & 0xc0) |
					     ((uint16_t)frame[4] << 8));
		if (highAmp >= 576 && highAmp > highPeak)
			highPeak = highAmp;
		if (lowAmp >= 576 && lowAmp > lowPeak)
			lowPeak = lowAmp;
	}
	*lowOut = lowPeak;
	*highOut = highPeak;
}

static void sw2Rumble(const uint8_t *payload, uint16_t size)
{
	if (!payload || size < 32 || !g_usbMountCount)
		return;
	int bond = g_usbToBond[0];
	if (bond < 0 || bond >= NSLOT)
		return;

	uint16_t leftLow = 0, leftHigh = 0, rightLow = 0, rightHigh = 0;
	sw2RumbleMotorBands(payload, &leftLow, &leftHigh);
	sw2RumbleMotorBands(payload + 16, &rightLow, &rightHigh);
	if (!leftLow && !leftHigh && !rightLow && !rightHigh) {
		if (!g_sw2RumbleActive && g_sw2LastRumbleBond < 0)
			return;
		g_sw2RumbleActive = false;
		if (g_switch2Hd) {
			g_sw2LastRumbleBond = -1;
			hapticSwitchHd((uint8_t)bond, 0, 0, 0, 0);
			return;
		}
		if (hapticSteamRumble(0, 0, (uint8_t)bond)) {
			g_sw2LastRumbleBond = -1;
			g_sw2LastRumbleLeft = 0;
			g_sw2LastRumbleRight = 0;
		}
		return;
	}
	g_sw2RumbleActive = true;
	if (g_switch2Hd) {
		g_sw2LastRumbleBond = -1;
		hapticSwitchHd((uint8_t)bond, leftLow, leftHigh, rightLow,
			       rightHigh);
		return;
	}

	uint16_t left = leftHigh > leftLow ? leftHigh : leftLow;
	uint16_t right = rightHigh > rightLow ? rightHigh : rightLow;
	if (bond == g_sw2LastRumbleBond && left == g_sw2LastRumbleLeft &&
	    right == g_sw2LastRumbleRight)
		return;

	if (hapticSteamRumble(left, right, (uint8_t)bond)) {
		g_sw2LastRumbleBond = (int8_t)bond;
		g_sw2LastRumbleLeft = left;
		g_sw2LastRumbleRight = right;
	}
}

extern "C" void __real_tud_hid_set_report_cb(uint8_t itf, uint8_t reportId,
					     hid_report_type_t reportType,
					     uint8_t const *buffer,
					     uint16_t size);
extern "C" void __wrap_tud_hid_set_report_cb(uint8_t itf, uint8_t reportId,
					     hid_report_type_t reportType,
					     uint8_t const *buffer,
					     uint16_t size)
{
	if (g_usbMode == MODE_SW_PRO2 && itf == 0) {
		// TinyUSB's hidd_xfer_cb passes HID_REPORT_TYPE_INVALID with
		// reportId == 0 for Interrupt OUT packets on endpoint 0x01.
		if (reportType == HID_REPORT_TYPE_OUTPUT ||
		    reportType == HID_REPORT_TYPE_INVALID) {
			if (reportId == 0x02) {
				sw2Rumble(buffer, size);
			} else if (reportId == 0 && size > 0 &&
				   buffer[0] == 0x02) {
				sw2Rumble(buffer + 1, (uint16_t)(size - 1));
			}
		}
		return;
	}
	__real_tud_hid_set_report_cb(itf, reportId, reportType, buffer, size);
}

void SwitchPro2Controller::begin()
{
}
uint8_t SwitchPro2Controller::maxSlots() const
{
	return 1;
}
void SwitchPro2Controller::usbIdentity()
{
	USBDevice.setID(0x057e, 0x2069);
	USBDevice.setVersion(0x0200);
	USBDevice.setDeviceVersion(0x0201);
	USBDevice.setManufacturerDescriptor("Nintendo");
	USBDevice.setProductDescriptor("Switch 2 Pro Controller");
	USBDevice.setSerialDescriptor("00");
}
void SwitchPro2Controller::beginPool()
{
	if (!g_sw2DrainRegistered) {
		usbTxRegisterDrain(sw2Drain);
		g_sw2DrainRegistered = true;
	}
}
void SwitchPro2Controller::mountSlots(uint8_t k)
{
	if (k)
		USBDevice.addInterface(g_sw2Usb);
}
void SwitchPro2Controller::task()
{
	// Report and vendor transfers are drained by usbTxPump() so endpoint
	// submission shares the same priority-inversion protection as other modes.
}
