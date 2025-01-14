/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_LPS331AP_H__
#define __AP_BARO_LPS331AP_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

/** Abstract serial device driver for LPS331AP. */
class AP_Baro_LPS331AP_Serial
{
public:
    /** Initialize the driver. */
    virtual void init() = 0;

    /** Read/write a 8-bit value from register "reg". */
    //virtual uint16_t read_16bits(uint8_t reg) = 0;
	virtual uint8_t read_reg(const uint8_t reg, uint8_t *const data) = 0;
	virtual uint8_t write_reg(const uint8_t reg, const uint8_t val) = 0;

	virtual uint8_t read_press(uint32_t *const pressure_raw) = 0;
	virtual uint8_t read_temp(int16_t *const temp_raw) = 0;
	
	virtual uint8_t readDeviceID() = 0;
    
    
    /** Read a 24-bit value from the ADC. */
    //virtual uint32_t read_adc() = 0;

    /** Write a single byte command. */
    //virtual void write(uint8_t reg) = 0;

    /** Acquire the internal semaphore for this device.
     * take_nonblocking should be used from the timer process,
     * take_blocking from synchronous code (i.e. init) */
    virtual bool sem_take_nonblocking() { return true; }
    virtual bool sem_take_blocking() { return true; }

    /** Release the internal semaphore for this device. */
    virtual void sem_give() {}
};

#if 0
/** SPI serial device. */
class AP_Baro_LPS331AP_SPI : public AP_Baro_LPS331AP_Serial
{
public:
    virtual void init();
    virtual uint16_t read_16bits(uint8_t reg);
    virtual uint32_t read_adc();
    virtual void write(uint8_t reg);
    virtual bool sem_take_nonblocking();
    virtual bool sem_take_blocking();
    virtual void sem_give();

private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};
#endif

/** I2C serial device. */
class AP_Baro_LPS331AP_I2C : public AP_Baro_LPS331AP_Serial
{
public:
    virtual void init();
    //virtual uint16_t read_16bits(uint8_t reg);
	virtual uint8_t read_reg(const uint8_t reg, uint8_t *const data);
	virtual uint8_t write_reg(const uint8_t reg, const uint8_t val);

	virtual uint8_t readDeviceID();
	virtual uint8_t read_press(uint32_t *const pressure_raw);
	virtual uint8_t read_temp(int16_t *const temp_raw);

   // virtual uint32_t read_adc();
    //virtual void write(uint8_t reg);
    virtual bool sem_take_nonblocking();
    virtual bool sem_take_blocking();
    virtual void sem_give();

private:
    AP_HAL::Semaphore *_i2c_sem;
};

class AP_Baro_LPS331AP : public AP_Baro
{
public:
    AP_Baro_LPS331AP(AP_Baro_LPS331AP_Serial *serial)
    {
        _serial = serial;
    }

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees

    /* Serial port drivers to pass to "init". */
    //static AP_Baro_LPS331AP_SPI spi;
    static AP_Baro_LPS331AP_I2C i2c;

private:
    /* Asynchronous handler functions: */
    void                            _update();
	
	void _update_temperature(void);
	void _update_pressure(void);
	
    /* Asynchronous state: */
    static volatile bool            _updated;
    //static volatile uint8_t         _d1_count;
    //static volatile uint8_t         _d2_count;
    //static volatile uint32_t        _s_D1, _s_D2;
	
	static uint32_t volatile _pressure_sum;
	static int16_t volatile _temperature;
	static uint8_t volatile _pressure_count;
	
    //static uint8_t                  _state;
    static uint32_t                 _timer;
    static AP_Baro_LPS331AP_Serial   *_serial;
    /* Gates access to asynchronous state: */
    static bool                     _sync_access;

    float                           Temp;
    float                           Press;

    uint32_t                        _raw_press;
    int32_t                         _raw_temp;
    // Internal calibration registers
    //uint16_t                        C1,C2,C3,C4,C5,C6;
    float                           D1,D2;

};

#endif //  __AP_BARO_LPS331AP_H__
