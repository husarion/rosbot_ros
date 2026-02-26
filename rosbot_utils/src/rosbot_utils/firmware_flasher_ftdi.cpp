// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libftdi1/ftdi.h>
#include <libusb-1.0/libusb.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

class FtdiCbus {
public:
  void open() {
    ctx_ = ftdi_new();
    if (!ctx_)
      throw std::runtime_error("ftdi_new failed");

    if (ftdi_usb_open(ctx_, 0x0403, 0x6015) < 0) {
      auto err = std::string(ftdi_get_error_string(ctx_));
      ftdi_free(ctx_);
      ctx_ = nullptr;
      throw std::runtime_error("ftdi_usb_open: " + err);
    }
    direction_ = value_ = 0;
  }

  void set_cbus_direction(uint8_t mask, uint8_t dir) {
    direction_ = (direction_ & ~mask) | (dir & mask);
    apply();
  }

  void set_cbus_gpio(uint8_t val) {
    value_ = val;
    apply();
  }

  void close() {
    if (ctx_) {
      // Disable bitbang
      ftdi_set_bitmode(ctx_, 0, 0);

      // Return the device to the kernel
      if (ctx_->usb_dev) {
        libusb_release_interface(ctx_->usb_dev, ctx_->interface);
        libusb_attach_kernel_driver(ctx_->usb_dev, ctx_->interface);
      }

      ftdi_usb_close(ctx_);
      ftdi_free(ctx_);
      ctx_ = nullptr;
    }
  }

  ~FtdiCbus() { close(); }

private:
  struct ftdi_context *ctx_ = nullptr;
  uint8_t direction_ = 0;
  uint8_t value_ = 0;

  void apply() {
    uint8_t bitmask = ((direction_ & 0x0F) << 4) | (value_ & 0x0F);
    if (ftdi_set_bitmode(ctx_, bitmask, BITMODE_CBUS) < 0)
      throw std::runtime_error("ftdi_set_bitmode: " +
                               std::string(ftdi_get_error_string(ctx_)));
  }
};

static void sleep_ms(int ms) { usleep(ms * 1000); }

static void run(const std::string &cmd) {
  if (std::system(cmd.c_str()) != 0)
    throw std::runtime_error("Command failed: " + cmd);
}

static void rebind_ftdi_driver() {
  int ret = std::system("usbreset 0403:6015 >/dev/null");
  (void)ret;
}

static void wait_for_device(const std::string &port, int timeout_s = 5) {
  for (int i = 0; i < timeout_s * 10; i++) {
    if (access(port.c_str(), F_OK) == 0) {
      sleep_ms(200); // give the kernel a moment to set permissions
      return;
    }
    sleep_ms(100);
  }
  throw std::runtime_error("Timeout waiting for " + port);
}

// ==========================================

static void enter_bootloader(FtdiCbus &f, const std::string &port) {
  f.open();
  f.set_cbus_direction(0b11, 0b11);
  sleep_ms(100);
  f.set_cbus_gpio(0b11); // BOOT0=1  RST=1
  sleep_ms(100);
  f.set_cbus_gpio(0b01); // BOOT0=1  RST=0 (reset release)
  sleep_ms(500);
  f.close();

  rebind_ftdi_driver();
  wait_for_device(port);
}

static void exit_bootloader(FtdiCbus &f, const std::string &port) {
  f.open();
  f.set_cbus_direction(0b11, 0b11);
  sleep_ms(100);
  f.set_cbus_gpio(0b10); // BOOT0=0  RST=1
  sleep_ms(300);
  f.set_cbus_gpio(0b00); // BOOT0=0  RST=0
  sleep_ms(100);
  f.set_cbus_direction(0b11, 0b00); // both → input (hi-Z)
  sleep_ms(100);
  f.close();

  rebind_ftdi_driver();
  wait_for_device(port);
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::fprintf(stderr, "Usage: %s <binary> <port>\n", argv[0]);
    return 1;
  }

  const std::string binary = argv[1];
  const std::string port = argv[2];

  std::printf("\nUSB Flashing:\n    File: %s\n    Port: %s\n", binary.c_str(),
              port.c_str());

  try {
    FtdiCbus ftdi;

    enter_bootloader(ftdi, port);

    std::printf("\nFlashing operation started\n");
    sleep_ms(500);
    run("stm32flash -b 115200 -v -w " + binary + " " + port);
    std::printf("Success\n");
    sleep_ms(500);

    exit_bootloader(ftdi, port);

  } catch (const std::exception &e) {
    std::fprintf(stderr, "Error: %s\n", e.what());
    std::fprintf(stderr, "You can restore USB port using: `sudo modprobe -r "
                         "ftdi_sio && sudo modprobe ftdi_sio`\n");
    return 1;
  }
  return 0;
}
