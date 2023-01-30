/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2011 ST-Ericsson SA
 * Written on behalf of Linaro for ST-Ericsson
 *
 * Author: Linus Walleij <linus.walleij@linaro.org>
 */

#ifndef _DT_BINDINGS_PINCTRL_PINCONF_GENERIC_H
#define _DT_BINDINGS_PINCTRL_PINCONF_GENERIC_H

#define BIAS_BUS_HOLD			0
#define BIAS_DISABLE			1
#define BIAS_HIGH_IMPEDANCE		2
#define BIAS_PULL_DOWN			3
#define BIAS_PULL_PIN_DEFAULT		4
#define BIAS_PULL_UP			5
#define DRIVE_OPEN_DRAIN		6
#define DRIVE_OPEN_SOURCE		7
#define DRIVE_PUSH_PULL			8
#define DRIVE_STRENGTH			9
#define DRIVE_STRENGTH_UA		10
#define INPUT_DEBOUNCE			11
#define INPUT_ENABLE			12
#define INPUT_SCHMITT			13
#define INPUT_SCHMITT_ENABLE		14
#define MODE_LOW_POWER			15
#define MODE_PWM			16
#define OUTPUT				17
#define OUTPUT_ENABLE			18
#define OUTPUT_IMPEDANCE_OHMS		19
#define PERSIST_STATE			20
#define POWER_SOURCE			21
#define SKEW_DELAY			22
#define SLEEP_HARDWARE_STATE		23
#define SLEW_RATE			24
#define PIN_CONFIG_END			0x7F
#define PIN_CONFIG_MAX			0xFF

#endif
