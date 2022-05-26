// SPDX-License-Identifier: BSD-1-Clause
/*
 * Open a serial port on Linux
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <termios.h>

namespace LS01 {
    int open_serial(const char *port, speed_t baud_option);
}

