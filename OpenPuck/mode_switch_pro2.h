// mode_switch_pro2.h -- Nintendo Switch 2 Pro Controller (057E:2069).
#pragma once
#include "controllers.h"
#include <stdint.h>

class SwitchPro2Controller : public IController {
    public:
	void begin() override;
	void task() override;
	bool dynamicMount() const override
	{
		return true;
	}
	uint8_t maxSlots() const override;
	void usbIdentity() override;
	void beginPool() override;
	void mountSlots(uint8_t k) override;
};

extern SwitchPro2Controller g_switchPro2;
