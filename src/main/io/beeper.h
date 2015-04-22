/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

void beeper(uint8_t mode);
void beeperUpdate(void);
void queueConfirmationBeep(uint8_t beepCount);
uint32_t getArmingBeepTimeMicros(void);

/* Beeper different modes: (lower number is higher priority)
 * BEEPER_STOP - Stops beeping
 * BEEPER_TX_LOST_ARMED - Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
 * BEEPER_TX_LOST - Beeps when TX is turned off or signal lost (repeat until TX is okay)
 * BEEPER_DISARMING - Beep when disarming the board
 * BEEPER_ARMING - Beep when arming the board
 * BEEPER_ARMING_GPS_FIX - Beep a tone when arming the board and GPS has fix
 * BEEPER_BAT_CRIT_LOW - Faster warning beeps when battery is critically low (repeats)
 * BEEPER_BAT_LOW - Warning beeps when battery is getting low (repeats)
 * BEEPER_TX_SET - Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled.
 * BEEPER_DISARM_REPEAT - Beeps sounded while stick held in disarm position
 * BEEPER_ACC_CALIBRATION - ACC inflight calibration completed confirmation
 * BEEPER_ACC_CALIBRATION_FAIL - ACC inflight calibration failed
 * BEEPER_READY_BEEP - Ring a tone when board is ready to flight (GPS ready).
 * BEEPER_CONFIRM_BEEP - Single short confirmation beep.
 * BEEPER_MULTI_BEEPS - Internal value used by 'queueConfirmationBeep()'.
 * BEEPER_ARMED - Warning beeps when board is armed. (repeats until board is disarmed or throttle is increased)
 */
enum {
    BEEPER_STOP = 0, // Highest priority command which is used only for stopping the beeper
    BEEPER_TX_LOST_ARMED,
    BEEPER_TX_LOST,
    BEEPER_DISARMING,
    BEEPER_ARMING,
    BEEPER_ARMING_GPS_FIX,
    BEEPER_BAT_CRIT_LOW,
    BEEPER_BAT_LOW,
    BEEPER_TX_SET,
    BEEPER_DISARM_REPEAT,
    BEEPER_ACC_CALIBRATION,
    BEEPER_ACC_CALIBRATION_FAIL,
    BEEPER_READY_BEEP,
    BEEPER_CONFIRM_BEEP,
    BEEPER_MULTI_BEEPS,
    BEEPER_ARMED,
    BEEPER_STOPPED // State which is used when beeper is in idle mode
};
