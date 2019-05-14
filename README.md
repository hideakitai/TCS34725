# TCS34725

Arduino library for TCS34725 RGB Color Sensor

## Description

This library is partially ported from this [circuitpython library](https://github.com/adafruit/Adafruit_CircuitPython_TCS34725).

- optimized performance (no suspend caused by ```delay()```)
- easily check if measurement cycle has done
- lux and color temperature calculation (ported from [here](https://github.com/adafruit/Adafruit_CircuitPython_TCS34725))
- interrupt feature is automatically enabled
- various parameter adjustment through APIs

## Usage

``` C++
#include "TCS34725.h"
TCS34725 tcs;

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();
    if (!tcs.attach(Wire))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");

    tcs.integrationTime(33); // ms
    tcs.gain(TCS34725::Gain::X01);

    // set LEDs...
}

void loop(void)
{
    if (tcs.available()) // if current measurement has done
    {
        TCS34725::Color color = tcs.color();
        Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs.lux());
        Serial.print("R          : "); Serial.println(color.r);
        Serial.print("G          : "); Serial.println(color.g);
        Serial.print("B          : "); Serial.println(color.b);
    }
}
```

## Parameter Adjustment

### Integration Time

Integration time for RGBC measurement can be set in millisecond.
The parameter range is from 2.4ms to 614.4ms.

``` C++
    void integrationTime(float ms) // 2.4 - 614.4 ms
```

### Gain

The gain of TCS can be set like:

``` C++
    tcs.gain(TCS34725::Gain::X01);
```

and you can choose gains as follows.

``` C++
    enum class Gain : uint8_t { X01, X04, X16, X60 };
```

### RGB Scaling

When RGB color is calculated, simple scaling by pow() is applied.

``` C++
    rgb_clr = pow(raw_rgb / raw_clear, scaling) * 255.f;
```

Default scaling parameter is 2.4, and if you want to disable it, please set to 1.0.

``` C++
    tcs.scale(2.5); // default
    tcs.scale(1.0); // disable scaling
```

### Glass Attenuation

This parameter is applied when lux and color temperature is calculated.
Please see [this library](https://github.com/adafruit/Adafruit_CircuitPython_TCS34725) for detail.

``` C++
    // The Glass Attenuation (FA) factor used to compensate for lower light
    // levels at the device due to the possible presence of glass. The GA is
    // the inverse of the glass transmissivity (T), so GA = 1/T. A transmissivity
    // of 50% gives GA = 1 / 0.50 = 2. If no glass is present, use GA = 1.
    // See Application Note: DN40-Rev 1.0 Ã¢â‚¬â€œ Lux and CCT Calculations using
    // ams Color Sensors for more details.
    tcs.glassAttenuation(1.0); // default: no glass
    tcs.glassAttenuation(0.5); // glass factor 50%
```

## APIs

``` C++
    struct Color { float r, g, b; };
    struct RawData { float r, g, b, c; };

    bool attach(WireType& w = Wire)
    void power(bool b)
    void enableColorTempAndLuxCalculation(bool b)

    void integrationTime(float ms) // 2.4 - 614.4 ms
    void gain(Gain g)
    void scale(float s)
    void glassAttenuation(float v)
    void persistence(uint16_t data)

    bool available()

    const RawData& raw() const
    const Color& color() const
    float lux() const
    float colorTemperature() const

    void interrupt(bool b)
    void clearInterrupt()

    // to manipulate registers, please use these APIs
    void write8(Reg reg, uint8_t value)
    uint8_t read8(Reg reg)
    uint16_t read16(Reg reg)
```

## Raw Register Manipulation

``` C++
    enum class Reg : uint8_t
    {
        ENABLE = 0x00,
        ATIME = 0x01,
        WTIME = 0x03,
        AILTL = 0x04,
        AILTH = 0x05,
        AIHTL = 0x06,
        AIHTH = 0x07,
        PERS = 0x0C,
        CONFIG = 0x0D,
        CONTROL = 0x0F,
        ID = 0x12,
        STATUS = 0x13,
        CDATAL = 0x14,
        CDATAH = 0x15,
        RDATAL = 0x16,
        RDATAH = 0x17,
        GDATAL = 0x18,
        GDATAH = 0x19,
        BDATAL = 0x1A,
        BDATAH = 0x1B,
    };

    enum class Mask : uint8_t
    {
        ENABLE_AIEN = 0x10,
        ENABLE_WEN = 0x08,
        ENABLE_AEN = 0x02,
        ENABLE_PON = 0x01,
        STATUS_AINT = 0x10,
        STATUS_AVALID = 0x01
    };
```


## License

MIT
