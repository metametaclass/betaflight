/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "uv.h"
#include "stdint.h"
#include "platform.h"
#include "rx/rx.h"
/*
typedef struct {
    double timestamp;                   // in seconds
    double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
    double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame
    double position_xyz[3];             // meters, NED from origin
} fdm_packet;

typedef struct {
    float motor_speed[4];   // normal: [0.0, 1.0], 3D: [-1.0, 1.0]
} servo_packet;
*/

typedef struct sitl2_state_s {
    //event loop
    uv_loop_t loop;

    //stdin
    uv_tty_t tty;

    //run timer
    uv_timer_t timer;

    //idle callback for tight loop scheduling
    uv_idle_t idle;

    //loop start 'real' time, nanoseconds
    uint64_t start_hrtime;

    //time in nanoseconds on previous step
    uint64_t time_prev_ns;

    //0 - 'real' time, 1 - simulated time
    int is_simulated_time;

    //how many time steps perform in run timer
    uint32_t steps_count;

    //simulated time in nanoseconds
    uint64_t sim_time_ns;

    //simulated time step, in nanoseconds
    uint64_t sim_time_step_ns;

    //simulation time call count
    uint64_t sim_timer_calls;

    //nanoseconds in simulation timer call
    uint64_t sim_timer_ns;

    //scheduler call count
    uint64_t scheduler_calls;

    //nanoseconds in scheduler call
    uint64_t scheduler_nanoseconds;

    //current rc channel values
    uint16_t rc_channels[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    //motor output
    uint16_t motor_speed[MAX_SUPPORTED_MOTORS];   // normal: [0.0, 1.0], 3D: [-1.0, 1.0]

} sitl2_state_t;

extern sitl2_state_t simulator_state;


//initialize simulator state
int sitl2_init(sitl2_state_t *state);

//cleanup simulator state
int sitl2_finit(sitl2_state_t *state);

//run simulator event loop
int sitl2_run(sitl2_state_t *state);

//close event loop and exit
void sitl2_close_event_loop(sitl2_state_t *state);

//run betaflight scheduler and calculate stats
void sitl2_scheduler_with_stats(sitl2_state_t *state);

//current time in nanoseconds
uint64_t sitl2_current_time_ns(sitl2_state_t *state);

//current time in microseconds
int64_t sitl2_current_time_us(sitl2_state_t *state);

//current time in milliseconds
int64_t sitl2_current_time_ms(sitl2_state_t *state);

//start working in simulated time
int sitl2_start_simulated_time(sitl2_state_t *state);

//stop working in simulated time
int sitl2_stop_simulated_time(sitl2_state_t *state);

//set rc channel value
int sitl2_set_rc_channel(sitl2_state_t *state, int channel, uint16_t value);

//send rc channels frame to betaflight
int sitl2_send_rc_channels(sitl2_state_t *state);
