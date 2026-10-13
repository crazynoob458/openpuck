// mode_switch_pro2.h -- Nintendo Switch 2 Pro Controller (057E:2069).
#pragma once
#include "controllers.h"
#include <stdint.h>

class SwitchPro2Controller : public IController {
    public:
	void begin() override;
	void task() override;
	void usbIdentity() override;
};

extern SwitchPro2Controller g_switchPro2;

void swPro2BulkCommand(const uint8_t *req, uint8_t len);
