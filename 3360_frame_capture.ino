/*
 * PMW3360 Frame Capture - reads the 36x36 grayscale image from the sensor
 * and sends it over USB serial.
 *
 * Based on 3360_Mouse_pico.ino by wareya (Apache-2.0)
 *
 * NOTE: Frame capture overwrites the SROM firmware. A full reset + SROM
 * reload is performed after each capture to restore the sensor.
 */

#include <SPI.h>

#include "srom_3360_0x05.h"

#define REG_PRODUCT_ID (0x00)
#define REG_REVISION_ID (0x01)
#define REG_MOTION (0x02)
#define REG_SQUAL (0x07)
#define REG_RAW_DATA_SUM (0x08)
#define REG_MAX_RAW_DATA (0x09)
#define REG_MIN_RAW_DATA (0x0A)
#define REG_CONFIG1 (0x0F)
#define REG_CONFIG2 (0x10)
#define REG_FRAME_CAPTURE (0x12)
#define REG_SROM_ENABLE (0x13)
#define REG_OBSERVATION (0x24)
#define REG_SROM_ID (0x2A)
#define REG_POWER_UP_RESET (0x3A)
#define REG_INVERSE_PRODUCT_ID (0x3F)
#define REG_SROM_LOAD_BURST (0x62)
#define REG_RAW_DATA_BURST (0x64)

#define PIN_NCS 17
#define PIN_MOSI 19
#define PIN_MISO 16

SPISettings spisettings(2000000, MSBFIRST, SPI_MODE3);

void spi_begin()
{
    SPI.beginTransaction(spisettings);
}

void spi_end()
{
    SPI.endTransaction();
}

void spi_write(byte addr, byte data)
{
    spi_begin();
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    SPI.transfer(addr | 0x80);
    SPI.transfer(data);
    delayMicroseconds(35);
    digitalWrite(PIN_NCS, HIGH);
    spi_end();
    delayMicroseconds(180);
}

byte spi_read(byte addr)
{
    spi_begin();
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    SPI.transfer(addr & 0x7F);
    delayMicroseconds(160);
    byte ret = SPI.transfer(0);
    delayMicroseconds(1);
    digitalWrite(PIN_NCS, HIGH);
    spi_end();
    delayMicroseconds(20);
    return ret;
}

void srom_upload()
{
    spi_write(REG_CONFIG2, 0x00);
    spi_write(REG_SROM_ENABLE, 0x1D);
    delay(10);
    spi_write(REG_SROM_ENABLE, 0x18);

    spi_begin();
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    SPI.transfer(REG_SROM_LOAD_BURST | 0x80);
    delayMicroseconds(15);
    for (size_t i = 0; i < SROM_LENGTH; i++)
    {
        SPI.transfer(srom[i]);
        delayMicroseconds(15);
    }
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);
    spi_end();
    delayMicroseconds(200);

    spi_write(REG_CONFIG2, 0x00);
}

void pmw3360_boot()
{
    spi_write(REG_POWER_UP_RESET, 0x5A);
    delay(50);
    spi_read(0x02);
    spi_read(0x03);
    spi_read(0x04);
    spi_read(0x05);
    spi_read(0x06);

    srom_upload();
    delay(10);

    spi_write(REG_CONFIG1, 11); // 1200 dpi
}

void print_sensor_status()
{
    byte product_id = spi_read(REG_PRODUCT_ID);
    byte inv_product_id = spi_read(REG_INVERSE_PRODUCT_ID);
    byte srom_id = spi_read(REG_SROM_ID);
    byte observation = spi_read(REG_OBSERVATION);
    byte squal = spi_read(REG_SQUAL);
    byte raw_sum = spi_read(REG_RAW_DATA_SUM);
    byte raw_max = spi_read(REG_MAX_RAW_DATA);
    byte raw_min = spi_read(REG_MIN_RAW_DATA);

    Serial.print("Product ID: 0x"); Serial.println(product_id, HEX);
    Serial.print("Inv Product ID: 0x"); Serial.println(inv_product_id, HEX);
    Serial.print("SROM ID: "); Serial.println(srom_id);
    Serial.print("Observation: 0x"); Serial.println(observation, HEX);
    Serial.print("SQUAL: "); Serial.println(squal);
    Serial.print("Raw data sum: "); Serial.println(raw_sum);
    Serial.print("Raw max: "); Serial.println(raw_max);
    Serial.print("Raw min: "); Serial.println(raw_min);
}

// capture one frame and send over serial
// mode 'f' = single capture with SROM reload after
// mode 'c' = fast capture, skip SROM reload (for streaming)
void pmw3360_frame_capture(bool reload_srom)
{
    // Datasheet steps 3-6:
    spi_write(REG_CONFIG2, 0x00);       // 3. Disable Rest mode
    spi_write(REG_FRAME_CAPTURE, 0x83); // 4. Write 0x83
    spi_write(REG_FRAME_CAPTURE, 0xC5); // 5. Write 0xC5
    delay(20);                          // 6. Wait 20ms

    // 7. Burst read from Raw_Data_Burst
    spi_begin();
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);

    SPI.transfer(REG_RAW_DATA_BURST & 0x7F);
    delayMicroseconds(160);

    Serial.write("FRAME", 5);
    for (int i = 0; i < 1296; i++)
    {
        byte pixel = SPI.transfer(0);
        Serial.write(pixel & 0x7F);
        delayMicroseconds(15);
    }
    Serial.write("END", 3);
    Serial.flush();

    delayMicroseconds(1);
    digitalWrite(PIN_NCS, HIGH);
    spi_end();

    if (reload_srom)
    {
        // full reset + SROM reload to restore navigation
        pmw3360_boot();
        delay(250);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);

    SPI.begin();
    pinMode(PIN_NCS, OUTPUT);

    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);

    pmw3360_boot();
    delay(250); // datasheet step 2: wait 250ms after power-up

    Serial.println("=== PMW3360 Frame Capture ===");
    print_sensor_status();
    Serial.println("'f' = single frame, 'c' = stream (fast), 's' = status");
}

bool streaming = false;

void loop()
{
    if (Serial.available())
    {
        char cmd = Serial.read();
        if (cmd == 'f')
        {
            streaming = false;
            pmw3360_frame_capture(true);  // with SROM reload
        }
        else if (cmd == 'c')
        {
            streaming = true;
        }
        else if (cmd == 'x')
        {
            streaming = false;
            // restore sensor after streaming
            pmw3360_boot();
            delay(250);
        }
        else if (cmd == 's')
        {
            print_sensor_status();
        }
    }

    if (streaming)
    {
        pmw3360_frame_capture(false);  // skip SROM reload
    }
}
