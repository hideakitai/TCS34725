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

        uint16_t r, g, b, c;
        tcs.raw(r, g, b, c);
        Serial.print("Raw R      : "); Serial.println(r);
        Serial.print("Raw G      : "); Serial.println(g);
        Serial.print("Raw B      : "); Serial.println(b);
        Serial.print("Raw C      : "); Serial.println(c);

        prev_ms = millis();
    }
}
