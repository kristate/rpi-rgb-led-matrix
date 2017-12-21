// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "gpio.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

namespace rgb_matrix {
/*static*/ const uint32_t GPIO::kValidBits
= ((1 <<  0) | (1 <<  1) | // RPi 1 - Revision 1 accessible
   (1 <<  2) | (1 <<  3) | // RPi 1 - Revision 2 accessible
   (1 <<  4) | (1 <<  7) | (1 << 8) | (1 <<  9) |
   (1 << 10) | (1 << 11) | (1 << 14) | (1 << 15)| (1 <<17) | (1 << 18) |
   (1 << 22) | (1 << 23) | (1 << 24) | (1 << 25)| (1 << 27) |
   // support for A+/B+ and RPi2 with additional GPIO pins.
   (1 <<  5) | (1 <<  6) | (1 << 12) | (1 << 13) | (1 << 16) |
   (1 << 19) | (1 << 20) | (1 << 21) | (1 << 26)
);

GPIO::GPIO() : output_bits_(0), slowdown_(1), gpio_port_(NULL) {
}

uint32_t GPIO::InitOutputs(uint32_t outputs) {
  if (gpio_port_ == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }
  return output_bits_;
}

// Based on code example found in http://elinux.org/RPi_Low-level_peripherals
bool GPIO::Init(int slowdown) {
#if 0
  slowdown_ = slowdown;
  gpio_port_ = mmap_bcm_register(IsRaspberryPi2(), GPIO_REGISTER_OFFSET);
  if (gpio_port_ == NULL) {
    return false;
  }
  gpio_set_bits_ = gpio_port_ + (0x1C / sizeof(uint32_t));
  gpio_clr_bits_ = gpio_port_ + (0x28 / sizeof(uint32_t));
#endif
  fprintf(stderr, "GPIO::Init\n");
  return true;
}

// --- PinPulser. Private implementation parts.
namespace {
// Manual timers.
class Timers {
public:
  static bool Init();
  static void sleep_nanos(long t);
};

// Simplest of PinPulsers. Uses somewhat jittery and manual timers
// to get the timing, but not optimal.
class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, uint32_t bits,
                      const std::vector<int> &nano_specs)
    : io_(io), bits_(bits), nano_specs_(nano_specs) {}

  virtual void SendPulse(int time_spec_number) {
    fprintf(stderr, "SendPulse\n");
  }

};

bool Timers::Init() {
  fprintf(stderr, "Timers::Init\n");
  return true;
}

void Timers::sleep_nanos(long nanos) {
  fprintf(stderr, "Timers::sleep_nanos\n");
}

// A PinPulser that uses the PWM hardware to create accurate pulses.
// It only works on GPIO-18 though.
class HardwarePinPulser : public PinPulser {
public:
  static bool CanHandle(uint32_t gpio_mask) {
    fprintf(stderr, "CanHandle\n");
    return true;
  }

  HardwarePinPulser(uint32_t pins, const std::vector<int> &specs)
    : triggered_(false) {
      fprintf(stderr, "HardwarePinPulser\n");
  }

  virtual void SendPulse(int c) {
    fprintf(stderr, "SendPulse\n");
  }

  virtual void WaitPulseFinished() {
    fprintf(stderr, "WaitPulseFinished\n");
  }

private:
  void SetGPIOMode(volatile uint32_t *gpioReg, unsigned gpio, unsigned mode) {
    fprintf(stderr, "SetGPIOMode\n");
  }

  void InitPWMDivider(uint32_t divider) {
    assert(divider < (1<<12));  // we only have 12 bits.
    fprintf(stderr, "InitPWMDivider\n");
  }

private:

};

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, uint32_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (!Timers::Init()) return NULL;
  if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
    return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  } else {
    return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
  }
}
} // namespace rgb_matrix
