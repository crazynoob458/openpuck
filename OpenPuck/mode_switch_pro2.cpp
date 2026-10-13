#include "mode_switch_pro2.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "rf_link.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

SwitchPro2Controller g_switchPro2;

// SC2 accel is +/-2g (16384/g); /4 -> the +/-8g (4096/g) scale Switch 2 expects
#define SW2_ACCEL_DIV 4
#define SWPRO2_CONN_MS 1200u

// Genuine Switch 2 Pro Controller (057E:2069) 97-byte USB HID report descriptor
// Declares Input 0x05 (63B vendor), Input 0x09 (63B gamepad), and Output 0x02
// (63B HD rumble).
static const uint8_t SWPRO2_HID_DESC[] = {
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x05, 0x05, 0xFF, 0x09,
	0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x95, 0x3F, 0x75, 0x08, 0x81,
	0x02, 0x85, 0x09, 0x09, 0x01, 0x95, 0x02, 0x81, 0x02, 0x05, 0x09,
	0x19, 0x01, 0x29, 0x15, 0x25, 0x01, 0x95, 0x15, 0x75, 0x01, 0x81,
	0x02, 0x95, 0x01, 0x75, 0x03, 0x81, 0x03, 0x05, 0x01, 0x09, 0x01,
	0xA1, 0x00, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x35, 0x26,
	0xFF, 0x0F, 0x95, 0x04, 0x75, 0x0C, 0x81, 0x02, 0xC0, 0x05, 0xFF,
	0x09, 0x02, 0x26, 0xFF, 0x00, 0x95, 0x34, 0x75, 0x08, 0x81, 0x02,
	0x85, 0x02, 0x09, 0x01, 0x95, 0x3F, 0x91, 0x02, 0xC0
};
static_assert(sizeof SWPRO2_HID_DESC == 97, "Switch 2 Pro HID descriptor size");

// Report 0x05 32-bit button bitfield (payload out[4..7], wire data[5..8])
#define SW2_05_Y (1u << 0)
#define SW2_05_X (1u << 1)
#define SW2_05_B (1u << 2)
#define SW2_05_A (1u << 3)
#define SW2_05_R (1u << 6)
#define SW2_05_ZR (1u << 7)
#define SW2_05_MINUS (1u << 8)
#define SW2_05_PLUS (1u << 9)
#define SW2_05_RSTICK (1u << 10)
#define SW2_05_LSTICK (1u << 11)
#define SW2_05_HOME (1u << 12)
#define SW2_05_CAPTURE (1u << 13)
#define SW2_05_C (1u << 14)
#define SW2_05_DOWN (1u << 16)
#define SW2_05_UP (1u << 17)
#define SW2_05_RIGHT (1u << 18)
#define SW2_05_LEFT (1u << 19)
#define SW2_05_L (1u << 22)
#define SW2_05_ZL (1u << 23)
#define SW2_05_GR (1u << 24)
#define SW2_05_GL (1u << 25)

// Report 0x09 24-bit button bitfield (payload out[2..4], wire data[3..5])
#define SW2_09_B (1u << 0)
#define SW2_09_A (1u << 1)
#define SW2_09_Y (1u << 2)
#define SW2_09_X (1u << 3)
#define SW2_09_R (1u << 4)
#define SW2_09_ZR (1u << 5)
#define SW2_09_PLUS (1u << 6)
#define SW2_09_RSTICK (1u << 7)
#define SW2_09_DOWN (1u << 8)
#define SW2_09_RIGHT (1u << 9)
#define SW2_09_LEFT (1u << 10)
#define SW2_09_UP (1u << 11)
#define SW2_09_L (1u << 12)
#define SW2_09_ZL (1u << 13)
#define SW2_09_MINUS (1u << 14)
#define SW2_09_LSTICK (1u << 15)
#define SW2_09_HOME (1u << 16)
#define SW2_09_CAPTURE (1u << 17)
#define SW2_09_GR (1u << 18)
#define SW2_09_GL (1u << 19)
#define SW2_09_C (1u << 20)

static uint32_t codeToSw2_05(uint8_t c, uint32_t fA, uint32_t fB, uint32_t fX,
			     uint32_t fY)
{
	switch (c) {
	case 1:
		return fA;
	case 2:
		return fB;
	case 3:
		return fX;
	case 4:
		return fY;
	case 5:
		return SW2_05_L;
	case 6:
		return SW2_05_R;
	case 7:
		return SW2_05_LSTICK;
	case 8:
		return SW2_05_RSTICK;
	case 9:
		return SW2_05_MINUS;
	case 10:
		return SW2_05_PLUS;
	case 11:
		return SW2_05_HOME;
	case 12:
		return SW2_05_UP;
	case 13:
		return SW2_05_DOWN;
	case 14:
		return SW2_05_LEFT;
	case 15:
		return SW2_05_RIGHT;
	case 18:
		return SW2_05_CAPTURE;
	case 19:
		return SW2_05_ZL;
	case 20:
		return SW2_05_ZR;
	case 21:
		return SW2_05_GL;
	case 22:
		return SW2_05_GR;
	case 23:
		return SW2_05_C;
	default:
		return 0;
	}
}

static uint32_t codeToSw2_09(uint8_t c, uint32_t fA, uint32_t fB, uint32_t fX,
			     uint32_t fY)
{
	switch (c) {
	case 1:
		return fA;
	case 2:
		return fB;
	case 3:
		return fX;
	case 4:
		return fY;
	case 5:
		return SW2_09_L;
	case 6:
		return SW2_09_R;
	case 7:
		return SW2_09_LSTICK;
	case 8:
		return SW2_09_RSTICK;
	case 9:
		return SW2_09_MINUS;
	case 10:
		return SW2_09_PLUS;
	case 11:
		return SW2_09_HOME;
	case 12:
		return SW2_09_UP;
	case 13:
		return SW2_09_DOWN;
	case 14:
		return SW2_09_LEFT;
	case 15:
		return SW2_09_RIGHT;
	case 18:
		return SW2_09_CAPTURE;
	case 19:
		return SW2_09_ZL;
	case 20:
		return SW2_09_ZR;
	case 21:
		return SW2_09_GL;
	case 22:
		return SW2_09_GR;
	case 23:
		return SW2_09_C;
	default:
		return 0;
	}
}

static Adafruit_USBD_HID g_swPro2Hid;
static unsigned long g_sw2LastMs = 0;

// Power-up defaults per commands.md: report 0x09 until host selects 0x05 via
// 0x03/0x0A. Stream enabled once 0x03/0x0D or 0x03/0x03 completes.
static volatile uint8_t g_sw2ReportId = 0x09;
static volatile bool g_sw2StreamEnabled = false;
static volatile uint8_t g_sw2FeatureMask = 0x27;
static volatile uint8_t g_sw2FeaturesEnabled = 0x27;

static uint32_t g_sw2Counter = 0;

// Non-zero start so SDL's `if (sensor_timestamp && !ctx->sensors_ready)` gate
// enters calibration on the first packet.
static uint32_t g_sw2SensorTs = 4000u;

// First bonded slot with a recent RF reply, or -1 if none is live.
static int swPro2LiveSlot()
{
	unsigned long now = millis();
	for (int s = 0; s < NSLOT; s++)
		if (g_slot[s].used && g_connReplyMs[s] &&
		    (now - g_connReplyMs[s]) < SWPRO2_CONN_MS)
			return s;
	return -1;
}

static uint8_t swPro2InputSlot()
{
	int s = swPro2LiveSlot();
	if (s >= 0)
		return (uint8_t)s;
	for (int i = 0; i < NSLOT; i++)
		if (g_slot[i].used)
			return (uint8_t)i;
	return 0;
}

static uint16_t sw2BatteryMv(uint8_t bond)
{
	uint8_t pct = (bond < NSLOT && g_battery[bond]) ? g_battery[bond] : 100;
	if (pct > 100)
		pct = 100;
	return (uint16_t)(3300u + (uint32_t)pct * 9u);
}

static uint8_t sw2ChargeStatus(uint8_t bond)
{
	return (bond < NSLOT && g_batteryState[bond] == 2) ? 0x34 : 0x20;
}

static int sw2Stick12(int16_t v)
{
	int a = 2048 + ((int)v >> 4);
	return a < 0 ? 0 : (a > 4095 ? 4095 : a);
}

static void sw2PackStick(uint8_t s[3], int16_t x, int16_t y)
{
	int X = sw2Stick12(x), Y = sw2Stick12(y);
	s[0] = (uint8_t)(X & 0xFF);
	s[1] = (uint8_t)(((Y & 0x0F) << 4) | ((X >> 8) & 0x0F));
	s[2] = (uint8_t)((Y >> 4) & 0xFF);
}

// Switch 2 stick calibration packs [x.neutral, y.neutral, x.max, y.max,
// x.min, y.min] as 6 12-bit values into 9 bytes for BOTH sticks (unlike
// Switch 1, which permuted left vs right).
static uint8_t g_sw2StickCal[9];
static void sw2BuildStickCal()
{
	const uint16_t v[6] = { 2048, 2048, 1800, 1800, 1800, 1800 };
	g_sw2StickCal[0] = v[0] & 0xFF;
	g_sw2StickCal[1] = ((v[1] & 0x0F) << 4) | ((v[0] >> 8) & 0x0F);
	g_sw2StickCal[2] = (v[1] >> 4) & 0xFF;
	g_sw2StickCal[3] = v[2] & 0xFF;
	g_sw2StickCal[4] = ((v[3] & 0x0F) << 4) | ((v[2] >> 8) & 0x0F);
	g_sw2StickCal[5] = (v[3] >> 4) & 0xFF;
	g_sw2StickCal[6] = v[4] & 0xFF;
	g_sw2StickCal[7] = ((v[5] & 0x0F) << 4) | ((v[4] >> 8) & 0x0F);
	g_sw2StickCal[8] = (v[5] >> 4) & 0xFF;
}

static void sw2Build05(uint8_t bond, uint8_t out[63])
{
	memset(out, 0, 63);
	uint32_t cnt = ++g_sw2Counter;
	out[0] = (uint8_t)(cnt & 0xFF);
	out[1] = (uint8_t)((cnt >> 8) & 0xFF);
	out[2] = (uint8_t)((cnt >> 16) & 0xFF);
	out[3] = (uint8_t)((cnt >> 24) & 0xFF);

	uint32_t b = g_in[bond].buttons | padDpadButtons(g_in[bond]);
	bool qam = g_qamMap && (b & TB_QAM);
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP | TB_DDN |
				 TB_DLF | TB_DRT);

	uint32_t fA = g_abSwap ? SW2_05_B : SW2_05_A;
	uint32_t fB = g_abSwap ? SW2_05_A : SW2_05_B;
	uint32_t fX = g_abSwap ? SW2_05_Y : SW2_05_X;
	uint32_t fY = g_abSwap ? SW2_05_X : SW2_05_Y;
	uint32_t btn = 0;

	if (b & TB_Y)
		btn |= fY;
	if (b & TB_B)
		btn |= fB;
	if (b & TB_A)
		btn |= fA;
	if (b & TB_X)
		btn |= fX;
	if (b & TB_LB)
		btn |= SW2_05_L;
	if (b & TB_RB)
		btn |= SW2_05_R;
	if ((g_in[bond].lt >= SW_TRIG_ON) || (b & TB_L2))
		btn |= SW2_05_ZL;
	if ((g_in[bond].rt >= SW_TRIG_ON) || (b & TB_R2))
		btn |= SW2_05_ZR;
	if (b & TB_VIEW)
		btn |= SW2_05_PLUS;
	if (b & TB_MENU)
		btn |= SW2_05_MINUS;
	if (b & TB_L3)
		btn |= SW2_05_LSTICK;
	if (b & TB_R3)
		btn |= SW2_05_RSTICK;
	if (b & TB_STEAM)
		btn |= SW2_05_HOME;
	if (b & TB_DDN)
		btn |= SW2_05_DOWN;
	if (b & TB_DUP)
		btn |= SW2_05_UP;
	if (b & TB_DRT)
		btn |= SW2_05_RIGHT;
	if (b & TB_DLF)
		btn |= SW2_05_LEFT;
	if (b & TB_L4)
		btn |= codeToSw2_05(g_back[0], fA, fB, fX, fY);
	if (b & TB_R4)
		btn |= codeToSw2_05(g_back[1], fA, fB, fX, fY);
	if (b & TB_L5)
		btn |= codeToSw2_05(g_back[2], fA, fB, fX, fY);
	if (b & TB_R5)
		btn |= codeToSw2_05(g_back[3], fA, fB, fX, fY);
	if (qam)
		btn |= codeToSw2_05(g_qamMap, fA, fB, fX, fY);

	out[4] = (uint8_t)(btn & 0xFF);
	out[5] = (uint8_t)((btn >> 8) & 0xFF);
	out[6] = (uint8_t)((btn >> 16) & 0xFF);
	out[7] = (uint8_t)((btn >> 24) & 0xFF);

	int16_t lx, ly, rx, ry;
	slotSticks(bond, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 10, lx, ly);
	sw2PackStick(out + 13, rx, ry);

	uint16_t mv = sw2BatteryMv(bond);
	out[31] = (uint8_t)(mv & 0xFF);
	out[32] = (uint8_t)(mv >> 8);
	out[33] = sw2ChargeStatus(bond);
	out[41] = 0x01;

	// SDL calibrates sensor_ts_coeff over 100 packets assuming a 4ms
	// (4000us) step; advancing by 4000u yields coeff=1000000 -> 34.8f.
	g_sw2SensorTs += 4000u;
	if (g_sw2SensorTs == 0)
		g_sw2SensorTs = 4000u;
	out[42] = (uint8_t)(g_sw2SensorTs & 0xFF);
	out[43] = (uint8_t)((g_sw2SensorTs >> 8) & 0xFF);
	out[44] = (uint8_t)((g_sw2SensorTs >> 16) & 0xFF);
	out[45] = (uint8_t)((g_sw2SensorTs >> 24) & 0xFF);

	int16_t aX = (int16_t)(g_in[bond].ay / SW2_ACCEL_DIV);
	int16_t aY = (int16_t)((-(int16_t)g_in[bond].ax) / SW2_ACCEL_DIV);
	int16_t aZ = (int16_t)(g_in[bond].az / SW2_ACCEL_DIV);

	int16_t groll = (int16_t)g_in[bond].gy;
	int16_t gpitch = (int16_t)(-(int16_t)g_in[bond].gx);
	int16_t gyaw = (int16_t)g_in[bond].gz;

	if (!g_swGyroLegacy) {
		groll = (int16_t)(((int32_t)groll * 4) / 5);
		gpitch = (int16_t)(((int32_t)gpitch * 9) / 10);
		gyaw = (int16_t)(((int32_t)gyaw * 9) / 10);
	}

	out[48] = (uint8_t)(aX & 0xFF);
	out[49] = (uint8_t)((aX >> 8) & 0xFF);
	out[50] = (uint8_t)(aY & 0xFF);
	out[51] = (uint8_t)((aY >> 8) & 0xFF);
	out[52] = (uint8_t)(aZ & 0xFF);
	out[53] = (uint8_t)((aZ >> 8) & 0xFF);

	out[54] = (uint8_t)(groll & 0xFF);
	out[55] = (uint8_t)((groll >> 8) & 0xFF);
	out[56] = (uint8_t)(gpitch & 0xFF);
	out[57] = (uint8_t)((gpitch >> 8) & 0xFF);
	out[58] = (uint8_t)(gyaw & 0xFF);
	out[59] = (uint8_t)((gyaw >> 8) & 0xFF);

	out[60] = g_in[bond].lt;
	out[61] = g_in[bond].rt;
}

static void sw2Build09(uint8_t bond, uint8_t out[63])
{
	memset(out, 0, 63);
	out[0] = (uint8_t)(++g_sw2Counter);

	uint8_t pct = (bond < NSLOT && g_battery[bond]) ? g_battery[bond] : 100;
	if (pct > 100)
		pct = 100;
	uint8_t lvl = (uint8_t)((pct * 9u + 50u) / 100u);
	if (lvl > 9)
		lvl = 9;
	uint8_t chg = (bond < NSLOT && g_batteryState[bond] == 2) ? 0x03 : 0x01;
	out[1] = (uint8_t)((lvl << 2) | chg);

	uint32_t b = g_in[bond].buttons | padDpadButtons(g_in[bond]);
	bool qam = g_qamMap && (b & TB_QAM);
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP | TB_DDN |
				 TB_DLF | TB_DRT);

	uint32_t fA = g_abSwap ? SW2_09_B : SW2_09_A;
	uint32_t fB = g_abSwap ? SW2_09_A : SW2_09_B;
	uint32_t fX = g_abSwap ? SW2_09_Y : SW2_09_X;
	uint32_t fY = g_abSwap ? SW2_09_X : SW2_09_Y;
	uint32_t btn = 0;

	if (b & TB_Y)
		btn |= fY;
	if (b & TB_B)
		btn |= fB;
	if (b & TB_A)
		btn |= fA;
	if (b & TB_X)
		btn |= fX;
	if (b & TB_LB)
		btn |= SW2_09_L;
	if (b & TB_RB)
		btn |= SW2_09_R;
	if ((g_in[bond].lt >= SW_TRIG_ON) || (b & TB_L2))
		btn |= SW2_09_ZL;
	if ((g_in[bond].rt >= SW_TRIG_ON) || (b & TB_R2))
		btn |= SW2_09_ZR;
	if (b & TB_VIEW)
		btn |= SW2_09_PLUS;
	if (b & TB_MENU)
		btn |= SW2_09_MINUS;
	if (b & TB_L3)
		btn |= SW2_09_LSTICK;
	if (b & TB_R3)
		btn |= SW2_09_RSTICK;
	if (b & TB_STEAM)
		btn |= SW2_09_HOME;
	if (b & TB_DDN)
		btn |= SW2_09_DOWN;
	if (b & TB_DUP)
		btn |= SW2_09_UP;
	if (b & TB_DRT)
		btn |= SW2_09_RIGHT;
	if (b & TB_DLF)
		btn |= SW2_09_LEFT;
	if (b & TB_L4)
		btn |= codeToSw2_09(g_back[0], fA, fB, fX, fY);
	if (b & TB_R4)
		btn |= codeToSw2_09(g_back[1], fA, fB, fX, fY);
	if (b & TB_L5)
		btn |= codeToSw2_09(g_back[2], fA, fB, fX, fY);
	if (b & TB_R5)
		btn |= codeToSw2_09(g_back[3], fA, fB, fX, fY);
	if (qam)
		btn |= codeToSw2_09(g_qamMap, fA, fB, fX, fY);

	out[2] = (uint8_t)(btn & 0xFF);
	out[3] = (uint8_t)((btn >> 8) & 0xFF);
	out[4] = (uint8_t)((btn >> 16) & 0xFF);

	int16_t lx, ly, rx, ry;
	slotSticks(bond, &lx, &ly, &rx, &ry);
	sw2PackStick(out + 5, lx, ly);
	sw2PackStick(out + 8, rx, ry);

	out[11] = (g_sw2FeaturesEnabled & 0x20) ? 0x38 : 0x30;
}

// --- Emulated 2MB flash memory map (memory_layout.md) ---
static const uint8_t SW2_FLASH_HDR0[48] = {
	0x01, 0x00, 0x64, 0xAA, 0x20, 0x53, 0x59, 0x53, 0xC6, 0x38, 0x8F, 0xE7,
	0x00, 0x00, 0x77, 0x10, 0x83, 0xFF, 0xEC, 0x07, 0x46, 0xEC, 0xF4, 0xD9,
	0x17, 0x2A, 0xE5, 0xAE, 0x0B, 0x5D, 0x6B, 0xCF, 0x12, 0x4A, 0xD5, 0x4C,
	0x4D, 0x83, 0xB0, 0x71, 0xBD, 0xE7, 0x01, 0xF2, 0xCC, 0xB5, 0x8B, 0x45
};

static const uint8_t SW2_FLASH_13000[37] = {
	0x01, 0x00, 'H',  'E',	'J',  '7',  '1',  '0',	'0',  '1',
	'1',  '2',  '1',  '2',	'4',  '7',  0x00, 0x00, 0x7E, 0x05,
	0x69, 0x20, 0x01, 0x06, 0x01, 0x23, 0x23, 0x23, 0xA0, 0xA0,
	0xA0, 0xE6, 0xE6, 0xE6, 0x32, 0x32, 0x32
};

// Bytes [4..15] are 3 IEEE-754 floats for gyro_bias_x/y/z; 0.0f prevents NaN
static const uint8_t SW2_FLASH_13040[16] = { 0x3B, 0xE0, 0xD3, 0x41, 0x00, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					     0x00, 0x00, 0x00, 0x00 };

static const uint8_t SW2_FLASH_13060[4] = { 0x4C, 0x09, 0x00, 0x00 };

static const uint8_t SW2_FLASH_STICK_PREFIX[40] = {
	0x01, 0xAD, 0xD9, 0x9A, 0x55, 0x56, 0x65, 0xA0, 0x00, 0x0A,
	0xA0, 0x00, 0x0A, 0xE2, 0x20, 0x0E, 0xE2, 0x20, 0x0E, 0x9A,
	0xAD, 0xD9, 0x9A, 0xAD, 0xD9, 0x0A, 0xA5, 0x50, 0x0A, 0xA5,
	0x50, 0x2F, 0xF6, 0x62, 0x2F, 0xF6, 0x62, 0x0A, 0xFF, 0xFF
};

static const uint8_t SW2_FLASH_13140[9] = { 0x00, 0xD7, 0xA3, 0xBC, 0x41,
					    0xD7, 0xA3, 0xBC, 0x41 };

static const uint8_t SW2_FLASH_13E00[32] = {
	'1',  '7',  '1',  '5',	'2',  '5',  '2',  '8',	'L',  'D',  '1',
	'7',  '4',  '3',  '1',	'0',  '1',  0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t SW2_FLASH_13E20[4] = { 0x01, 0x02, 0x02, 0x02 };

static const uint8_t SW2_FLASH_13E30[10] = { 0x03, 0x02, 0x04, 0x01, 0x05,
					     0x02, 0x06, 0x02, 0x07, 0x02 };

static const uint8_t SW2_FLASH_13E60[2] = { 0x11, 0x00 };

static const uint8_t SW2_FLASH_13E80[9] = { 0x00, 0x23, 0xE9, 0xCE, 0x00,
					    0x41, 0x30, 0x4B, 0x41 };

static const uint8_t SW2_FLASH_13EFB[4] = { 0x01, 0x00, 0x15, 0x0C };

// DSP firmware header at 0x175000 ("DSPH") read by the console during init
static const uint8_t SW2_FLASH_175000[64] = {
	0x44, 0x53, 0x50, 0x48, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x96, 0x02,
	0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x09, 0x10, 0xFD, 0x16,
	0x29, 0x8A, 0x06, 0x5F, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t SW2_FLASH_1FB000[8] = { 0x02, 0x00, 0x02, 0x00,
					     0x01, 0x00, 0x11, 0x03 };

static const uint8_t SW2_FLASH_1FB042[28] = {
	0x02, 0x00, 0x02, 0x00, 0xC8, 0x00, 0x78, 0x00, 0x99, 0x0E,
	0xD4, 0x0D, 0x4C, 0x0D, 0xD5, 0x0C, 0x90, 0x0C, 0x66, 0x0C,
	0xFD, 0x0B, 0xF7, 0x0B, 0xAA, 0x0B, 0xAA, 0x0A
};

static const uint8_t SW2_FLASH_1FB070[18] = { 0x99, 0x0E, 0xFF, 0x0B, 0xFF,
					      0xFF, 0x55, 0x09, 0xFF, 0xFF,
					      0x55, 0x0D, 0x55, 0x0C, 0x3B,
					      0x0C, 0x00, 0x00 };

static uint8_t g_sw2UserCal[0x100];
static bool g_sw2CalInit = false;

// Pairing info at 0x1FA000 (updated by 0x03/0x07, 0x03/0x08, or 0x15)
static uint8_t g_sw2PairCount = 0x02;
static uint8_t g_sw2HostAddr[6] = { 0x98, 0xE2, 0x55, 0x2E, 0x31, 0x4B };
static uint8_t g_sw2LtkRev[16] = { 0xD6, 0x2C, 0xC6, 0x34, 0xD3, 0x57,
				   0xA0, 0x48, 0xE9, 0x89, 0x6F, 0x98,
				   0x79, 0x6A, 0x2D, 0x71 };

static void sw2FlashRead(uint32_t addr, uint8_t len, uint8_t *dst)
{
	for (uint8_t i = 0; i < len; i++) {
		uint32_t a = (addr + i) & 0x1FFFFFu;
		uint8_t v = 0xFF;

		if (a < sizeof SW2_FLASH_HDR0)
			v = SW2_FLASH_HDR0[a];
		else if (a >= 0x15000 && a < 0x15000 + sizeof SW2_FLASH_HDR0)
			v = SW2_FLASH_HDR0[a - 0x15000];
		else if (a >= 0x13000 && a < 0x13000 + sizeof SW2_FLASH_13000)
			v = SW2_FLASH_13000[a - 0x13000];
		else if (a >= 0x13040 && a < 0x13040 + sizeof SW2_FLASH_13040)
			v = SW2_FLASH_13040[a - 0x13040];
		else if (a >= 0x13060 && a < 0x13060 + sizeof SW2_FLASH_13060)
			v = SW2_FLASH_13060[a - 0x13060];
		else if (a >= 0x13080 &&
			 a < 0x13080 + sizeof SW2_FLASH_STICK_PREFIX)
			v = SW2_FLASH_STICK_PREFIX[a - 0x13080];
		else if (a >= 0x130A8 && a < 0x130A8 + 9)
			v = g_sw2StickCal[a - 0x130A8];
		else if (a >= 0x130C0 &&
			 a < 0x130C0 + sizeof SW2_FLASH_STICK_PREFIX)
			v = SW2_FLASH_STICK_PREFIX[a - 0x130C0];
		else if (a >= 0x130E8 && a < 0x130E8 + 9)
			v = g_sw2StickCal[a - 0x130E8];
		else if (a >= 0x13100 && a < 0x13118)
			// 3 IEEE-754 floats at +12..+23 for accel_bias_x/y/z
			v = 0x00;
		else if (a >= 0x13140 && a < 0x13140 + sizeof SW2_FLASH_13140)
			v = SW2_FLASH_13140[a - 0x13140];
		else if (a >= 0x13E00 && a < 0x13E00 + sizeof SW2_FLASH_13E00)
			v = SW2_FLASH_13E00[a - 0x13E00];
		else if (a >= 0x13E20 && a < 0x13E20 + sizeof SW2_FLASH_13E20)
			v = SW2_FLASH_13E20[a - 0x13E20];
		else if (a >= 0x13E30 && a < 0x13E30 + sizeof SW2_FLASH_13E30)
			v = SW2_FLASH_13E30[a - 0x13E30];
		else if (a >= 0x13E60 && a < 0x13E60 + sizeof SW2_FLASH_13E60)
			v = SW2_FLASH_13E60[a - 0x13E60];
		else if (a >= 0x13E80 && a < 0x13E80 + sizeof SW2_FLASH_13E80)
			v = SW2_FLASH_13E80[a - 0x13E80];
		else if (a >= 0x13EFB && a < 0x13EFB + sizeof SW2_FLASH_13EFB)
			v = SW2_FLASH_13EFB[a - 0x13EFB];
		else if (a >= 0x175000 &&
			 a < 0x175000 + sizeof SW2_FLASH_175000)
			v = SW2_FLASH_175000[a - 0x175000];
		else if (a == 0x1FA000)
			v = g_sw2PairCount;
		else if (a >= 0x1FA008 && a < 0x1FA00E)
			v = g_sw2HostAddr[a - 0x1FA008];
		else if (a >= 0x1FA01A && a < 0x1FA02A)
			v = g_sw2LtkRev[a - 0x1FA01A];
		else if (a >= 0x1FA030 && a < 0x1FA035)
			v = g_sw2HostAddr[a - 0x1FA030];
		else if (a == 0x1FA035)
			v = (uint8_t)(g_sw2HostAddr[5] - 1u);
		else if (a >= 0x1FA042 && a < 0x1FA052)
			v = g_sw2LtkRev[a - 0x1FA042];
		else if (a >= 0x1FB000 &&
			 a < 0x1FB000 + sizeof SW2_FLASH_1FB000)
			v = SW2_FLASH_1FB000[a - 0x1FB000];
		else if (a >= 0x1FB00E && a < 0x1FB010)
			v = 0x00;
		else if (a >= 0x1FB042 &&
			 a < 0x1FB042 + sizeof SW2_FLASH_1FB042)
			v = SW2_FLASH_1FB042[a - 0x1FB042];
		else if (a >= 0x1FB070 &&
			 a < 0x1FB070 + sizeof SW2_FLASH_1FB070)
			v = SW2_FLASH_1FB070[a - 0x1FB070];
		else if (a >= 0x1FC000 && a < 0x1FC100)
			v = g_sw2UserCal[a - 0x1FC000];
		else if (a >= 0x1FD010 && a < 0x1FD014)
			v = 0x00;

		dst[i] = v;
	}
}

static void sw2FlashWrite(uint32_t addr, uint8_t len, const uint8_t *data,
			  uint8_t avail)
{
	for (uint8_t i = 0; i < len && i < avail; i++) {
		uint32_t a = (addr + i) & 0x1FFFFFu;
		if (a >= 0x1FC000 && a < 0x1FC100)
			g_sw2UserCal[a - 0x1FC000] = data[i];
	}
}

// Subcommand 0x15/0x02 confirms the LTK by encrypting the 16-byte challenge A2
// with AES-128-ECB under LTK = A1 ^ B1 (both transmitted in reverse order).
static const uint8_t SW2_PAIR_B1_REV[16] = { 0x5C, 0xF6, 0xEE, 0x79, 0x2C, 0xDF,
					     0x05, 0xE1, 0xBA, 0x2B, 0x63, 0x25,
					     0xC4, 0x1A, 0x5F, 0x10 };

static void sw2AesEcbChallenge(const uint8_t a2Rev[16], uint8_t b2Rev[16])
{
	struct {
		uint8_t key[16];
		uint8_t cleartext[16];
		uint8_t ciphertext[16];
	} ecb;

	for (int i = 0; i < 16; i++) {
		ecb.key[i] = g_sw2LtkRev[15 - i];
		ecb.cleartext[i] = a2Rev[15 - i];
	}
	NRF_ECB->ECBDATAPTR = (uint32_t)&ecb;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	NRF_ECB->TASKS_STARTECB = 1;
	uint32_t w = micros();
	while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB &&
	       (uint32_t)(micros() - w) < 1000u) {
	}
	for (int i = 0; i < 16; i++)
		b2Rev[i] = ecb.ciphertext[15 - i];
}

// Sends Bulk IN replies directly via tud_vendor_write/flush because SDL3 and
// the Switch 2 console never send the WebUSB 0x22 control transfer that sets
// usb_web.connected().
static void sw2SendBulk(const uint8_t *resp, uint8_t respLen)
{
	if (!tud_mounted() || !respLen)
		return;
	usbTxBoost();
	if (tud_vendor_write_available() >= respLen) {
		tud_vendor_write(resp, respLen);
		tud_vendor_flush();
	}
	usbTxUnboost();
}

void swPro2BulkCommand(const uint8_t *req, uint8_t len)
{
	if (!req || len < 8 || req[1] != 0x91)
		return;

	uint8_t cmd = req[0];
	uint8_t tr = req[2];
	uint8_t sub = req[3];
	const uint8_t *d = req + 8;
	uint8_t dlen = (len > 8) ? (uint8_t)(len - 8) : 0;

	static uint8_t resp[96];
	memset(resp, 0, sizeof resp);
	resp[0] = cmd;
	resp[1] = 0x01;
	resp[2] = tr;
	resp[3] = sub;
	resp[4] = 0x00;
	resp[5] = 0xF8;
	uint8_t respLen = 8;

	switch (cmd) {
	case 0x01: // NFC / Rumble init
		if (sub == 0x0C) {
			resp[4] = 0x10;
			resp[5] = 0x78;
			resp[8] = 0x61;
			resp[9] = 0x12;
			resp[10] = 0x50;
			resp[11] = 0x0D;
			respLen = 12;
		} else if (sub == 0x05) {
			resp[8] = 0x09;
			resp[12] = 0x01;
			resp[13] = 0x01;
			resp[14] = 0x02;
			respLen = 64;
		}
		break;

	case 0x02: // Flash memory
		if (sub == 0x01 && dlen >= 8) {
			uint32_t addr = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
					((uint32_t)d[6] << 16) |
					((uint32_t)d[7] << 24);
			resp[4] = 0x10;
			resp[5] = 0x78;
			resp[8] = 0x40;
			memcpy(resp + 12, d + 4, 4);
			sw2FlashRead(addr, 0x40, resp + 16);
			respLen = 80;
		} else if (sub == 0x04 && dlen >= 8) {
			uint8_t rl = d[0] > 0x50 ? 0x50 : d[0];
			uint32_t addr = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
					((uint32_t)d[6] << 16) |
					((uint32_t)d[7] << 24);
			resp[4] = 0x10;
			resp[5] = 0x78;
			resp[8] = rl;
			memcpy(resp + 12, d + 4, 4);
			sw2FlashRead(addr, rl, resp + 16);
			respLen = (uint8_t)(16 + rl);
		} else if ((sub == 0x02 || sub == 0x05) && dlen >= 8) {
			uint8_t wl = (sub == 0x02) ?
					     0x40 :
					     (d[0] > 0x4F ? 0x4F : d[0]);
			uint32_t addr = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
					((uint32_t)d[6] << 16) |
					((uint32_t)d[7] << 24);
			sw2FlashWrite(addr, wl, d + 8,
				      (dlen > 8) ? (uint8_t)(dlen - 8) : 0);
			resp[4] = 0x10;
			resp[5] = 0x78;
			memcpy(resp + 12, d + 4, 4);
			respLen = 16;
		} else if (sub == 0x03) {
			if (dlen >= 8) {
				uint32_t addr = (uint32_t)d[4] |
						((uint32_t)d[5] << 8) |
						((uint32_t)d[6] << 16) |
						((uint32_t)d[7] << 24);
				if ((addr & 0x1FF000u) == 0x1FC000u)
					memset(g_sw2UserCal, 0xFF,
					       sizeof g_sw2UserCal);
			}
			resp[4] = 0x00;
			resp[5] = 0x78;
			respLen = 12;
		}
		break;

	case 0x03: // Initialisation
		if (sub == 0x03) {
			g_sw2StreamEnabled = (dlen < 1) || (d[0] != 0);
			resp[8] = 0x01;
			respLen = 12;
		} else if (sub == 0x07) {
			if (dlen >= 22) {
				for (int i = 0; i < 6; i++)
					g_sw2HostAddr[i] = d[5 - i];
				memcpy(g_sw2LtkRev, d + 6, 16);
				g_sw2PairCount = 0x02;
			}
			resp[4] = 0x10;
			resp[5] = 0x78;
		} else if (sub == 0x08) {
			g_sw2PairCount = 0x00;
		} else if (sub == 0x09) {
			g_sw2PairCount = 0x02;
			resp[4] = 0x10;
			resp[5] = 0x78;
		} else if (sub == 0x0A) {
			if (dlen >= 1 && (d[0] == 0x05 || d[0] == 0x09))
				g_sw2ReportId = d[0];
		} else if (sub == 0x0D) {
			g_sw2StreamEnabled = true;
			resp[8] = 0x01;
			respLen = 12;
		} else if (sub == 0x0F) {
			resp[8] = 0x05;
			respLen = 12;
		}
		break;

	case 0x06:
		if (sub == 0x02)
			return; // Docked sleep shutdown sends no response
		resp[4] = 0x10;
		resp[5] = 0x78;
		break;

	case 0x07:
		if (sub == 0x01) {
			resp[4] = 0x10;
			resp[5] = 0x78;
			resp[8] = 0x00;
			respLen = 9;
		}
		break;

	case 0x08: // Charging grip
		if (sub == 0x01 || sub == 0x03) {
			resp[12] = 0x01;
			memcpy(resp + 14, SW2_FLASH_13000 + 2, 23);
			memset(resp + 37, 0xFF,
			       (sub == 0x03) ? (76 - 37) : (44 - 37));
			respLen = (sub == 0x03) ? 76 : 44;
		}
		break;

	case 0x09: // Player LEDs
	case 0x0A: // Vibration
	case 0x0D: // Firmware update stubs
	case 0x17:
		resp[4] = 0x10;
		resp[5] = 0x78;
		break;

	case 0x0B: { // Battery
		uint8_t bond = swPro2InputSlot();
		resp[4] = 0x10;
		resp[5] = 0x78;
		if (sub == 0x03) {
			uint16_t mv = sw2BatteryMv(bond);
			resp[8] = (uint8_t)(mv & 0xFF);
			resp[9] = (uint8_t)(mv >> 8);
			respLen = 12;
		} else if (sub == 0x04) {
			resp[8] = sw2ChargeStatus(bond);
			resp[10] = 0x83;
			respLen = 12;
		} else if (sub == 0x06) {
			resp[8] = 0x11;
			respLen = 12;
		}
		break;
	}

	case 0x0C: // Feature select
		resp[4] = 0x10;
		resp[5] = 0x78;
		if (sub == 0x01) {
			uint8_t f = (dlen >= 1) ? d[0] : 0x2F;
			resp[12] = (f & 0x01) ? 0x07 : 0x00;
			resp[13] = (f & 0x02) ? 0x07 : 0x00;
			resp[14] = (f & 0x04) ? 0x01 : 0x00;
			resp[15] = (f & 0x80) ? 0x01 : 0x00;
			resp[16] = (f & 0x10) ? 0x01 : 0x00;
			resp[17] = (f & 0x20) ? 0x03 : 0x00;
			respLen = 20;
		} else if (sub == 0x02) {
			if (dlen >= 1)
				g_sw2FeatureMask = d[0];
			respLen = 12;
		} else if (sub == 0x03) {
			g_sw2FeatureMask = 0;
			g_sw2FeaturesEnabled = 0;
			respLen = 12;
		} else if (sub == 0x04) {
			if (dlen >= 1)
				g_sw2FeaturesEnabled |=
					(uint8_t)(d[0] & g_sw2FeatureMask);
			respLen = 12;
		} else if (sub == 0x05) {
			if (dlen >= 1)
				g_sw2FeaturesEnabled &= (uint8_t)~d[0];
			respLen = 12;
		} else if (sub == 0x06) {
			resp[12] = 0x02;
			resp[17] = 0x50;
			respLen = 48;
		}
		break;

	case 0x10: // Firmware version info (type 0x02 = Pro Controller 2)
		if (sub == 0x01) {
			static const uint8_t FW_INFO[12] = { 0x01, 0x00, 0x0E,
							     0x02, 0x01, 0x0C,
							     0x00, 0x00, 0xFF,
							     0xFF, 0xFF, 0xFF };
			resp[4] = 0x10;
			resp[5] = 0x78;
			memcpy(resp + 8, FW_INFO, sizeof FW_INFO);
			respLen = 20;
		}
		break;

	case 0x11:
		resp[4] = 0x10;
		resp[5] = 0x78;
		if (sub == 0x01) {
			resp[8] = 0x01;
			respLen = 12;
		} else if (sub == 0x03) {
			static const uint8_t D11_03[29] = {
				0x01, 0x20, 0x03, 0x00, 0x00, 0x0A, 0xE8, 0x1C,
				0x3B, 0x79, 0x7D, 0x8B, 0x3A, 0x0A, 0xE8, 0x9C,
				0x42, 0x58, 0xA0, 0x0B, 0x42, 0x0A, 0xE8, 0x9C,
				0x41, 0x58, 0xA0, 0x0B, 0x41
			};
			memcpy(resp + 8, D11_03, sizeof D11_03);
			respLen = 37;
		}
		break;

	case 0x13:
		resp[4] = 0x10;
		resp[5] = 0x78;
		resp[8] = 0x01;
		respLen = (sub == 0x01) ? 12 : 16;
		break;

	case 0x15: // Bluetooth pairing
		resp[4] = 0x10;
		resp[5] = 0x78;
		if (sub == 0x01) {
			if (dlen >= 8 && d[1] >= 1) {
				for (int i = 0; i < 6; i++)
					g_sw2HostAddr[i] = d[2 + 5 - i];
			}
			static const uint8_t PAIR_ADDR[9] = {
				0x01, 0x04, 0x01, 0x88, 0x16,
				0xC2, 0x55, 0xE2, 0x98
			};
			memcpy(resp + 8, PAIR_ADDR, sizeof PAIR_ADDR);
			respLen = 17;
		} else if (sub == 0x04) {
			if (dlen >= 17) {
				for (int i = 0; i < 16; i++)
					g_sw2LtkRev[i] = d[1 + i] ^
							 SW2_PAIR_B1_REV[i];
			}
			resp[8] = 0x01;
			memcpy(resp + 9, SW2_PAIR_B1_REV, 16);
			respLen = 25;
		} else if (sub == 0x02) {
			resp[8] = 0x01;
			if (dlen >= 17)
				sw2AesEcbChallenge(d + 1, resp + 9);
			respLen = 25;
		} else if (sub == 0x03) {
			g_sw2PairCount = 0x02;
			resp[8] = 0x01;
			respLen = 9;
		}
		break;

	case 0x16:
		if (sub == 0x01) {
			resp[4] = 0x10;
			resp[5] = 0x78;
			respLen = 32;
		}
		break;

	case 0x18:
		resp[4] = 0x10;
		resp[5] = 0x78;
		if (sub == 0x01) {
			resp[10] = 0x40;
			resp[11] = 0xF0;
			resp[14] = 0x60;
			respLen = 16;
		} else if (sub == 0x03) {
			resp[8] = (dlen >= 1) ? d[0] : 0x07;
			respLen = 9;
		}
		break;

	default:
		break;
	}

	sw2SendBulk(resp, respLen);
}

// Inverse of SDL3's EncodeHDRumble (SDL_hidapi_switch2.c): 5 bytes per motor
// carrying 10-bit frequencies and 12-bit amplitudes scaled against RUMBLE_MAX.
static uint16_t sw2DecodeMotorAmp(const uint8_t m[5])
{
	uint16_t highAmp = ((uint16_t)(m[1] & 0xFC) << 4) |
			   ((uint16_t)(m[2] & 0x0F) << 12);
	uint16_t lowAmp = (uint16_t)(m[3] & 0xC0) | ((uint16_t)m[4] << 8);
	uint16_t raw = (lowAmp > highAmp) ? lowAmp : highAmp;
	if (!raw)
		return 0;
	uint32_t scaled = ((uint32_t)raw * 65535u) / 29000u;
	return (scaled > 0xFFFFu) ? 0xFFFFu : (uint16_t)scaled;
}

static uint16_t g_sw2LastLo = 0;
static uint16_t g_sw2LastHi = 0;
static bool g_sw2LastSet = false;

static void sw2SetReport(uint8_t rid, hid_report_type_t type, uint8_t const *b,
			 uint16_t n)
{
	if (type != HID_REPORT_TYPE_OUTPUT || !b || n < 1)
		return;
	uint8_t id;
	const uint8_t *p;
	uint16_t pn;
	if (rid == 0) {
		id = b[0];
		p = b + 1;
		pn = (uint16_t)(n - 1);
	} else {
		id = rid;
		p = b;
		pn = n;
	}
	if (id != 0x02 || pn < 22)
		return;

	int slot = swPro2LiveSlot();
	if (slot < 0)
		return;

	uint16_t lo = sw2DecodeMotorAmp(p + 1);
	uint16_t hi = sw2DecodeMotorAmp(p + 17);
	if (g_sw2LastSet && lo == g_sw2LastLo && hi == g_sw2LastHi)
		return;
	g_sw2LastSet = true;
	g_sw2LastLo = lo;
	g_sw2LastHi = hi;
	hapticSteamRumble(lo, hi, (uint8_t)slot);
}

static uint16_t sw2GetReport(uint8_t rid, hid_report_type_t type, uint8_t *buf,
			     uint16_t reqlen)
{
	if (type != HID_REPORT_TYPE_INPUT || !buf || reqlen < 63)
		return 0;
	uint8_t bond = swPro2InputSlot();
	uint8_t id = rid ? rid : g_sw2ReportId;
	if (id == 0x05)
		sw2Build05(bond, buf);
	else
		sw2Build09(bond, buf);
	return 63;
}

void SwitchPro2Controller::usbIdentity()
{
	USBDevice.setID(0x057E, 0x2069);
	USBDevice.setVersion(0x0200);
	USBDevice.setDeviceVersion(0x0200);
	USBDevice.setManufacturerDescriptor("Nintendo");
	USBDevice.setProductDescriptor("Switch 2 Pro Controller");
}

void SwitchPro2Controller::begin()
{
	sw2BuildStickCal();
	if (!g_sw2CalInit) {
		memset(g_sw2UserCal, 0xFF, sizeof g_sw2UserCal);
		g_sw2CalInit = true;
	}
	g_sw2ReportId = 0x09;
	g_sw2StreamEnabled = false;
	g_sw2FeatureMask = 0x27;
	g_sw2FeaturesEnabled = 0x27;
	g_sw2LastSet = false;

	usbIdentity();
	USBDevice.setConfigurationMaxPower(500);
	g_swPro2Hid.enableOutEndpoint(true);
	g_swPro2Hid.setReportCallback(sw2GetReport, sw2SetReport);
	g_swPro2Hid.setReportDescriptor(SWPRO2_HID_DESC,
					sizeof SWPRO2_HID_DESC);
	g_swPro2Hid.setPollInterval(1);
	g_swPro2Hid.begin();
}

void SwitchPro2Controller::task()
{
	if (!g_swPro2Hid.ready())
		return;
	if (!g_sw2StreamEnabled)
		return;
	if (millis() - g_sw2LastMs < USB_STREAM_MS)
		return;
	g_sw2LastMs = millis();

	uint8_t bond = swPro2InputSlot();
	uint8_t p[63];
	uint8_t rid = g_sw2ReportId;
	if (rid == 0x05)
		sw2Build05(bond, p);
	else
		sw2Build09(bond, p);
	usbTxHid(&g_swPro2Hid, rid, p, sizeof p);
}
