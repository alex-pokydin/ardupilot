/*
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

/*
 *       AP_RangeFinder_TWTOF240UI.cpp
 *       Code by Peter Barker
 *
 *       datasheet: https://myosuploads3.banggood.com/products/20190919/20190919025445TOF101201.zip
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_TWTOF240UI.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// TOF Module I2C slave address = 54 decimal.
#define SLAVE_ADDRESS 0x36

// I2C command codes.
#define TOFM_CMD_NONE            0x00
#define TOFM_CMD_START_FLAG      0x80
#define TOFM_CMD_ST_MM           0x81
#define TOFM_CMD_CALI_XTALK      0x82
#define TOFM_CMD_CALI_OFS        0x83
#define TOFM_CMD_RESET           0x84
#define TOFM_CMD_RD_FACTORY_DATA 0x85
#define TOFM_CMD_RD_VERSION_INFO 0x86
#define TOFM_CMD_RD_DEBUG_PARA1  0x8A
#define TOFM_CMD_RD_DEBUG_PARA2  0x8B
#define TOFM_CMD_RD_DEBUG_PARA3  0x8C


/*
   detect if a TWTOF240UI rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_TWTOF240UI::detect(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_TWTOF240UI *sensor
        = new AP_RangeFinder_TWTOF240UI(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_TWTOF240UI::_init(void)
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    if (!check_reading()) {
        return false;
    }

    _dev->register_periodic_callback(
        100000,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TWTOF240UI::_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_TWTOF240UI::check_reading()
{
    uint8_t buf[5];

    _dev->write_register(TOFM_CMD_START_FLAG, 0x01);
    hal.scheduler->delay_microseconds(10);

    _dev->write_register(TOFM_CMD_ST_MM, 0x01);
    hal.scheduler->delay_microseconds(10);

    if (!_dev->read_registers(TOFM_CMD_ST_MM, buf, 5)) {
        return false;
    }

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_TWTOF240UI::get_reading(uint16_t &reading_mm)
{
    uint8_t buf[5];

    _dev->write_register(TOFM_CMD_START_FLAG, 0x01);
    hal.scheduler->delay_microseconds(10);

    _dev->write_register(TOFM_CMD_ST_MM, 0x01);
    hal.scheduler->delay_microseconds(10);

    if (!_dev->read_registers(TOFM_CMD_ST_MM, buf, 5)) {
        return false;
    }

    uint8_t distHi       = buf[2];
    uint8_t distLo       = buf[3];

    reading_mm = distHi * 256 + distLo;
    
    // ignore failed readings
    if(reading_mm > 3000){
        return false;
    }

    return true;
}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_TWTOF240UI::_timer(void)
{

    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance_mm = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_TWTOF240UI::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance_mm * 0.001f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}