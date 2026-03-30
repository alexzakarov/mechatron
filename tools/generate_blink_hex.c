#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Intel HEX checksum calculation
uint8_t calc_checksum(const uint8_t* data, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (0x100 - (sum & 0xFF)) & 0xFF;
}

// Write Intel HEX record
void write_hex_record(FILE* f, uint8_t byte_count, uint16_t address,
                      uint8_t type, const uint8_t* data) {
    fprintf(f, ":%02X%04X%02X", byte_count, address, type);

    uint8_t checksum_data[4 + byte_count];
    checksum_data[0] = byte_count;
    checksum_data[1] = (address >> 8) & 0xFF;
    checksum_data[2] = address & 0xFF;
    checksum_data[3] = type;

    for (int i = 0; i < byte_count; i++) {
        fprintf(f, "%02X", data[i]);
        checksum_data[4 + i] = data[i];
    }

    uint8_t checksum = calc_checksum(checksum_data, 4 + byte_count);
    fprintf(f, "%02X\n", checksum);
}

int main() {
    FILE* f = fopen("blink_clean.hex", "w");

    // Arduino Blink Firmware for ATmega328P
    // LED on PB5 (Arduino pin 13)

    // Vector table at 0x0000
    // Reset vector: JMP to 0x0080
    uint8_t reset_vec[4] = {0x0C, 0x94, 0x80, 0x00};  // JMP 0x0080

    // Write reset vector
    write_hex_record(f, 4, 0x0000, 0x00, reset_vec);

    // Code at 0x0080 - Simple blink program
    // Each line is 16 bytes
    uint8_t code1[16] = {
        // Set PB5 as output: SER R24; OUT DDRB,R24
        0x80, 0xE0,        // SER R24 (R24 = 0xFF)
        0x84, 0xB9,        // OUT DDRB, R24 (all outputs)

        // LED ON: LDI R24, 0x20; OUT PORTB,R24
        0x82, 0xE0,        // LDI R24, 0x20 (PB5 mask)
        0x85, 0xB8,        // OUT PORTB, R24 (LED on)

        // Delay outer loop setup
        0xEF, 0xEF,        // LDI R30, 0xFF
        0xFF, 0xE0,        // LDI R31, 0x00

        // Delay middle loop setup
        0xEE, 0xEF,        // LDI R30, 0xFE (reuse as counter)
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00         // padding
    };

    uint8_t code2[16] = {
        // Inner delay loop (decrement and branch)
        0x96, 0x95,        // DEC R30
        0xF4, 0xF7,        // BRNE -4 (loop back)
        0x95, 0x95,        // DEC R31 (middle counter)
        0xF1, 0xF7,        // BRNE -6
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00         // padding
    };

    uint8_t code3[16] = {
        // LED OFF: LDI R24, 0x00; OUT PORTB,R24
        0x80, 0xE0,        // LDI R24, 0x00
        0x85, 0xB8,        // OUT PORTB, R24 (LED off)

        // Delay again
        0xEF, 0xEF,        // LDI R30, 0xFF
        0xFF, 0xE0,        // LDI R31, 0x00
        0xEE, 0xEF,        // LDI R30, 0xFE
        0x96, 0x95,        // DEC R30
        0xF4, 0xF7,        // BRNE -4
        0x95, 0x95,        // DEC R31
        0xF1, 0xF7,        // BRNE -6
        0x00, 0x00,        // padding
        0x00, 0x00         // padding
    };

    uint8_t code4[16] = {
        // Jump back to start
        0x0C, 0x94,        // JMP
        0x80, 0x00,        // address 0x0080
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00,        // padding
        0x00, 0x00         // padding
    };

    // Write code blocks
    write_hex_record(f, 16, 0x0080, 0x00, code1);
    write_hex_record(f, 16, 0x0090, 0x00, code2);
    write_hex_record(f, 16, 0x00A0, 0x00, code3);
    write_hex_record(f, 16, 0x00B0, 0x00, code4);

    // EOF
    write_hex_record(f, 0, 0x0000, 0x01, NULL);

    fclose(f);
    printf("Generated blink_clean.hex\n");

    return 0;
}
