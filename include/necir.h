/*  Copyright (C) 2020  Florian Menne
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NECIR_H_
#define NECIR_H_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes NECIR.
 */
void necir_init(void);

/**
 * Callback if valid data is received.
 * @param addr Address.
 * @param cmd Command - Mapped to individual buttons.
 * @param repeat Counts up every 100ms as long as the button is held on the remote.
 * @param idle True if signal gets idle (button released on remote).
 */
void necir_callback(uint16_t addr, uint8_t cmd, uint32_t repeat, bool idle);

#ifdef __cplusplus
}
#endif

#endif /* NECIR_H_ */
