#ifndef I2C_SCAN
#define I2C_SCAN

#include "Arduino.h"
#include "Wire.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int i2cscan(TwoWire * bus) {
    i2c_inst_t * instance;
    Serial.print("\nI2C");
    if (bus=&Wire) {
        instance=i2c0;
        Serial.print(0);
    } else {
        instance=i2c1;
        Serial.print(1);
    }

    Serial.print("Bus Scan\n");
    Serial.printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(instance, addr, &rxdata, 1, false);

        Serial.printf(ret < 0 ? "." : "@");
        Serial.printf(addr % 16 == 15 ? "\n" : "  ");
    }
    Serial.printf("Done.\n");
    return 0;
}

#endif