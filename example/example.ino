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
    if (tcs.available())
    {
        static uint32_t prev_ms = millis();

        TCS34725::Color color = tcs.color();
        Serial.print("Interval   : "); Serial.println(millis() - prev_ms);
        Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs.lux());
        Serial.print("R          : "); Serial.println(color.r);
        Serial.print("G          : "); Serial.println(color.g);
        Serial.print("B          : "); Serial.println(color.b);


        TCS34725::RawData raw = tcs.raw();
        Serial.print("Raw R      : "); Serial.println(raw.r);
        Serial.print("Raw G      : "); Serial.println(raw.g);
        Serial.print("Raw B      : "); Serial.println(raw.b);
        Serial.print("Raw C      : "); Serial.println(raw.c);

        prev_ms = millis();
    }
}
