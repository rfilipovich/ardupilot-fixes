/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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


#include <AP_HAL.h>
#include "AP_Baro_LPS331AP.h"

//#define LPS331_DEBUG 1

extern const AP_HAL::HAL& hal;



#define LPS_STATUS_ADDR         0x27

#define LPS_PRESS_OUT_XL_ADDR   0x28
#define LPS_PRESS_OUT_L_ADDR    0x29
#define LPS_PRESS_OUT_H_ADDR    0x2A

#define LPS_TEMP_OUT_L_ADDR     0x2B
#define LPS_TEMP_OUT_H_ADDR     0x2C

#define LPS_WHO_AM_I_ADDR      0x0F
#define LPS331_WHO_AM_I        0xBB
#define LPS_RES_CONF_ADDR 	   0x10

#define LPS_CTRL_REG1_ADDR      0x20
#define LPS_CTRL_REG1_ODR0_BIT  0x10
#define LPS_CTRL_REG1_ODR1_BIT  0x20
#define LPS_CTRL_REG1_ODR2_BIT  0x40
#define LPS_CTRL_REG1_PD_BIT    0x80

#define LPS_CTRL_REG2_ADDR      0x21
#define LPS_CTRL_REG3_ADDR      0x22


uint32_t volatile AP_Baro_LPS331AP::_pressure_sum;
int16_t volatile AP_Baro_LPS331AP::_temperature;
uint8_t volatile AP_Baro_LPS331AP::_pressure_count;

uint32_t AP_Baro_LPS331AP::_timer;
bool volatile AP_Baro_LPS331AP::_updated;

AP_Baro_LPS331AP_Serial* AP_Baro_LPS331AP::_serial = NULL;
AP_Baro_LPS331AP_I2C AP_Baro_LPS331AP::i2c;


// I2C Device //////////////////////////////////////////////////////////////////

/** I2C address of the LPS331AP. */
#define LPS331AP_ADDR 0x5C

void AP_Baro_LPS331AP_I2C::init()
{
    _i2c_sem = hal.i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP did not get "
                                  "valid I2C semaphroe!"));
        return; /* never reached */
    }
}

uint8_t AP_Baro_LPS331AP_I2C::read_reg(const uint8_t reg, uint8_t *const data)
{
    uint8_t buf;

    if (hal.i2c->readRegister(LPS331AP_ADDR, reg, &buf) == 0) {
		if(data) *data = buf;
        return 0;
	};

    return (uint8_t) (-1);
}

uint8_t AP_Baro_LPS331AP_I2C::write_reg(const uint8_t reg, const uint8_t val)
{
    if (hal.i2c->writeRegister(LPS331AP_ADDR, reg, val) == 0) {
        return 0;
	};

    return (uint8_t) (-1);
}

uint8_t AP_Baro_LPS331AP_I2C::read_press(uint32_t *const pressure_raw)
{
    uint8_t buf[3];
	uint32_t pressure;
	int retv = 0;

   //if (hal.i2c->readRegisters(LPS331AP_ADDR, LPS_PRESS_OUT_XL_ADDR, sizeof(buf), buf) == 0) {
   //     pressure = (((uint32_t)buf[2]) << 16) | (((uint16_t)buf[1]) << 8) | buf[0];
	//	if(pressure_raw) *pressure_raw = pressure;
	//	return 0;
   //};
	retv = read_reg(LPS_PRESS_OUT_XL_ADDR, &buf[0]);
	retv |= read_reg(LPS_PRESS_OUT_L_ADDR, &buf[1]);
	retv |= read_reg(LPS_PRESS_OUT_H_ADDR, &buf[2]);
	if(!retv) {
		pressure = (((uint32_t)buf[2]) << 16) | (((uint16_t)buf[1]) << 8) | buf[0];
		if(pressure_raw) *pressure_raw = pressure;
	};

	return 0;

    return (uint8_t) (-1);
}

uint8_t AP_Baro_LPS331AP_I2C::read_temp(int16_t *const temp_raw)
{
    uint8_t buf[2];
	int16_t temp;
	int retv = 0;

    //if (hal.i2c->readRegisters(LPS331AP_ADDR, LPS_TEMP_OUT_L_ADDR, sizeof(buf), buf) == 0) {
    //    temp = ((((uint16_t)buf[1]) << 8) | buf[0]);
	//	if(temp_raw) *temp_raw = temp;
	//	return 0;
	//}

	retv = read_reg(LPS_TEMP_OUT_L_ADDR, &buf[0]);
	retv |= read_reg(LPS_TEMP_OUT_H_ADDR, &buf[1]);
	if(!retv) {
		temp = ((((uint16_t)buf[1]) << 8) | buf[0]);
		if(temp_raw) *temp_raw = temp;
		return 0;
	};

   return (uint8_t) (-1);
}

uint8_t AP_Baro_LPS331AP_I2C::readDeviceID()
{
	uint8_t data = 0;
	read_reg(LPS_WHO_AM_I_ADDR, &data);
	return data;
}

//void AP_Baro_LPS331AP_I2C::write(uint8_t reg)
//{
//    hal.i2c->write(LPS331AP_ADDR, 1, &reg);
//}

bool AP_Baro_LPS331AP_I2C::sem_take_blocking() {
    return _i2c_sem->take(10);
}

bool AP_Baro_LPS331AP_I2C::sem_take_nonblocking()
{
    /**
     * Take nonblocking from a TimerProcess context &
     * monitor for bad failures
     */
    static int semfail_ctr = 0;
    bool got = _i2c_sem->take_nonblocking();
    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            semfail_ctr++;
            if (semfail_ctr > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _i2c_sem "
                                          "100 times in a row, in "
                                          "AP_Baro_LPS331AP::_update"));
            }
        }
        return false; /* never reached */
    } else {
        semfail_ctr = 0;
    }
    return got;
}

void AP_Baro_LPS331AP_I2C::sem_give()
{
    _i2c_sem->give();
}

// Public Methods //////////////////////////////////////////////////////////////

// SPI should be initialized externally
bool AP_Baro_LPS331AP::init()
{
	uint8_t _deviceID;
	uint8_t retv;

    if (_serial == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP: NULL serial driver"));
        return false; /* never reached */
    }

    _serial->init();
    if (!_serial->sem_take_blocking()){
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP: failed to take "
                    "serial semaphore for init"));
        return false; /* never reached */
    }


    //hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP was not detected on I2C bus"));
	_deviceID = _serial->readDeviceID();
    uint8_t data = 0;
    if (_deviceID != LPS331_WHO_AM_I) {
		hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP was not detected on I2C bus"));
    };

#if defined(LPS331_DEBUG)
	hal.console->printf("I am 0x%x\n", _deviceID);
#endif

    /* reset device */
	_serial->write_reg(LPS_CTRL_REG2_ADDR, (1 << 2));
	_serial->write_reg(LPS_CTRL_REG2_ADDR, 0x00);
	hal.scheduler->delay(4);

    _serial->write_reg(LPS_CTRL_REG1_ADDR, 0x00); // turn off for config
	_serial->write_reg(LPS_RES_CONF_ADDR, 0x6a); /* for the RMS noise  ~0.020 for barometar (25/25) */

    retv = _serial->write_reg(LPS_CTRL_REG1_ADDR,
			(LPS_CTRL_REG1_ODR0_BIT | LPS_CTRL_REG1_ODR1_BIT |
			LPS_CTRL_REG1_ODR2_BIT | LPS_CTRL_REG1_PD_BIT));
    if (retv) {
		hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP can't write to I2C bus"));
    };

#if defined(LPS331_DEBUG)
	{
		uint8_t reg = 0;
		_serial->read_reg(LPS_CTRL_REG1_ADDR, &reg);
		hal.console->printf("LPS_CTRL_REG1_ADDR=0x%x\n", reg);
		_serial->read_reg(LPS_RES_CONF_ADDR, &reg);
		hal.console->printf("LPS_RES_CONF_ADDR=0x%x\n", reg);
		
		
		//// test ////
		for(int i = 0; i < 0x30; i++) {
			uint8_t reg = 0;
			_serial->read_reg(i, &reg);
			hal.console->printf("0x%x:0x%x\n", i, reg);
		}
		
	}
#endif
	
    _timer = hal.scheduler->micros();

    Temp=0;
    Press=0;

    _pressure_sum  = 0;
    _temperature = 0;
    _pressure_count = 0;

    hal.scheduler->register_timer_process( AP_HAL_MEMBERPROC(&AP_Baro_LPS331AP::_update));
    _serial->sem_give();

    // wait for at least one value to be read
    uint32_t tstart = hal.scheduler->millis();
    while (!_updated) {
        hal.scheduler->delay(10);
        if (hal.scheduler->millis() - tstart > 1000) {
            hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331AP took more than "
                        "1000ms to initialize"));
            healthy = false;
            return false;
        }
    }

    healthy = true;
    return true;
}

// calculate temperature
void AP_Baro_LPS331AP::_update_temperature(void)
{
    int16_t Temp_Reg_s16;
	if (_serial->read_temp(&Temp_Reg_s16)) {
		return;
	}

	_temperature = Temp_Reg_s16;
}

// calculate pressure
void AP_Baro_LPS331AP::_update_pressure(void)
{
	uint32_t Pressure_Reg_u32;
	if (_serial->read_press(&Pressure_Reg_u32)) {
		return;
	}
    _pressure_sum += Pressure_Reg_u32;
    _pressure_count++;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void AP_Baro_LPS331AP::_update(void)
{
    uint32_t tnow = hal.scheduler->micros();
    // Throttle read rate to 25hz maximum.
    if (tnow - _timer < 40000) {
        return;
    }

    if (!_serial->sem_take_nonblocking()) {
        return;
    }
    _timer = tnow;

	uint8_t status;

    // use status to check if data is available
    if (_serial->read_reg(LPS_STATUS_ADDR, &status)) {
		/* reading with error */
        return;
    }

#if defined(LPS331_DEBUG)
	//hal.console->printf("Readed status:0x%x\n", status);
#endif

    if (status & 0x02) {
        _update_temperature();
    }

    if (status & 0x01) {
        _update_pressure();
        _updated = true;
	};

    _serial->sem_give();
}

/* getting raw presure */
uint8_t AP_Baro_LPS331AP::read()
{
    bool updated = _updated;

    if (updated) {
        uint32_t sD1;
        uint8_t d1count;

        hal.scheduler->suspend_timer_procs();
        sD1 = _pressure_sum; _pressure_sum = 0;
        D2 = _temperature;
        d1count = _pressure_count; _pressure_count = 0;

        _updated = false;
        hal.scheduler->resume_timer_procs();

        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }

        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }

	//Press = /*D1*/((_raw_press * 100.0f) / 4096); // scale for pa
	//Temp = (_raw_temp * (1.0/480)) + 42.5;
	Press = ((D1 * 100.0f) / 4096);
	Temp =  ((D2/480) + 42.5);

    if (updated) {
        _last_update = hal.scheduler->millis();
    }

    return updated ? 1 : 0;
}

float AP_Baro_LPS331AP::get_pressure()
{
    return Press; /* in Pa */
}

float AP_Baro_LPS331AP::get_temperature()
{
    // temperature in degrees C units
    return Temp;
}

