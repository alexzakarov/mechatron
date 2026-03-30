#include "ATmegaInterpreter.hpp"
#include "FirmwareLoader.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace mechatron {

// ATmega328P flash size: 32KB = 16K words
constexpr uint16_t FLASH_SIZE_WORDS = 16 * 1024;
constexpr uint32_t F_CPU = 16000000;  // 16 MHz

ATmegaInterpreter::ATmegaInterpreter(QEMUInterface& mcu)
    : m_mcu(mcu)
{
    m_flash.resize(FLASH_SIZE_WORDS, 0xFFFF);
    build_instruction_table();
}

void ATmegaInterpreter::build_instruction_table() {
    // Initialize all entries to NOP_other handler (for unknown instructions)
    for (auto& entry : m_instruction_table) {
        entry = &ATmegaInterpreter::exec_NOP_other;
    }

    // Build instruction dispatch table based on opcode patterns

    // NOP: 0000 0000 0000 0000
    m_instruction_table[0x0000] = &ATmegaInterpreter::exec_NOP;

    // MOV: 0010 11rd dddd rrrr (between registers) - 0x2C00 to 0x2FFF
    for (uint16_t op = 0x2C00; op <= 0x2FFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_MOV;
    }

    // AND: 0010 00rd dddd rrrr - 0x2000 to 0x23FF
    for (uint16_t op = 0x2000; op <= 0x23FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_AND;
    }

    // EOR: 0010 01rd dddd rrrr - 0x2400 to 0x27FF
    for (uint16_t op = 0x2400; op <= 0x27FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_EOR;
    }

    // OR: 0010 10rd dddd rrrr - 0x2800 to 0x2BFF
    for (uint16_t op = 0x2800; op <= 0x2BFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_OR;
    }

    // LDI: 1110 KKKK dddd KKKK (load immediate) - 0xE000 to 0xFFFF
    // KKKK is split: bits 11-8 (high) and bits 3-0 (low), dddd is bits 7-4
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0xE000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_LDI;
            }
        }
    }

    // Common single instructions
    m_instruction_table[0x9508] = &ATmegaInterpreter::exec_RET;
    m_instruction_table[0x9518] = &ATmegaInterpreter::exec_RETI;
    m_instruction_table[0x95C8] = &ATmegaInterpreter::exec_LPM;

    // STS: 1001 001d dddd 0000
    for (uint16_t op = 0x9200; op <= 0x9300; op += 0x100) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_STS;
    }

    // LDS: 1001 000d dddd 0000
    for (uint16_t op = 0x9000; op <= 0x9100; op += 0x100) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_LDS;
    }

    // ADD: 0000 11rd dddd rrrr
    for (uint16_t op = 0x0C00; op <= 0x0FFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_ADD;
    }

    // ADC: 0001 11rd dddd rrrr
    for (uint16_t op = 0x1C00; op <= 0x1FFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_ADC;
    }

    // SUB: 0001 10rd dddd rrrr
    for (uint16_t op = 0x1800; op < 0x1C00; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_SUB;
    }

    // SUBI: 0101 KKKK dddd KKKK
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0x5000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_SUBI;
            }
        }
    }

    // SBCI: 0100 KKKK dddd KKKK
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0x4000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_SBCI;
            }
        }
    }

    // ANDI: 0111 KKKK dddd KKKK
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0x7000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_ANDI;
            }
        }
    }

    // ORI: 0110 KKKK dddd KKKK
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0x6000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_ORI;
            }
        }
    }

    // CPI: 0011 KKKK dddd KKKK
    for (uint16_t k_high = 0; k_high < 16; k_high++) {
        for (uint16_t d = 0; d < 16; d++) {
            for (uint16_t k_low = 0; k_low < 16; k_low++) {
                uint16_t op = 0x3000 | (k_high << 8) | (d << 4) | k_low;
                m_instruction_table[op] = &ATmegaInterpreter::exec_CPI;
            }
        }
    }

    // RJMP: 1100 kkkk kkkk kkkk
    for (uint16_t op = 0xC000; op <= 0xCFFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_RJMP;
    }

    // RCALL: 1101 kkkk kkkk kkkk
    for (uint16_t op = 0xD000; op <= 0xDFFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_RCALL;
    }

    // CP: 0001 01rd dddd rrrr
    for (uint16_t op = 0x1400; op <= 0x17FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_CP;
    }

    // CPC: 0000 01rd dddd rrrr
    for (uint16_t op = 0x0400; op <= 0x07FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_CPC;
    }

    // Branch instructions - use fixed iteration count to avoid overflow
    for (int i = 0; i < 7; i++) {
        m_instruction_table[0xF001 + (i * 0x200)] = &ATmegaInterpreter::exec_BREQ;
    }
    for (int i = 0; i < 6; i++) {
        m_instruction_table[0xF401 + (i * 0x200)] = &ATmegaInterpreter::exec_BRNE;
    }
    for (int i = 0; i < 6; i++) {
        m_instruction_table[0xF400 + (i * 0x200)] = &ATmegaInterpreter::exec_BRSH;
    }
    for (int i = 0; i < 7; i++) {
        m_instruction_table[0xF000 + (i * 0x200)] = &ATmegaInterpreter::exec_BRLO;
    }
    for (int i = 0; i < 6; i++) {
        m_instruction_table[0xF402 + (i * 0x200)] = &ATmegaInterpreter::exec_BRMI;
    }
    for (int i = 0; i < 6; i++) {
        m_instruction_table[0xF403 + (i * 0x200)] = &ATmegaInterpreter::exec_BRPL;
    }

    // IN: 1011 0AAd dddd AAAA
    for (uint16_t op = 0xB000; op <= 0xB7FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_IN;
    }

    // OUT: 1011 1AAd dddd AAAA
    for (uint16_t op = 0xB800; op <= 0xBFFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_OUT;
    }

    // JMP: 1001 010k kkkk 110k / 1001 010k kkkk 111k (2-word instruction)
    // Just iterate all possible JMP first-word opcodes
    for (uint16_t h = 0x94; h <= 0x95; h++) {
        for (uint16_t l = 0x0C; l <= 0x0D; l++) {
            uint16_t op = (h << 8) | l;
            m_instruction_table[op] = &ATmegaInterpreter::exec_JMP;
        }
    }
    // Explicitly set common JMP opcodes (Arduino uses these)
    m_instruction_table[0x940C] = &ATmegaInterpreter::exec_JMP;
    m_instruction_table[0x950C] = &ATmegaInterpreter::exec_JMP;
    m_instruction_table[0x940D] = &ATmegaInterpreter::exec_JMP;
    m_instruction_table[0x950D] = &ATmegaInterpreter::exec_JMP;

    // IJMP: 1001 0100 0000 1110 (0x940E) - Indirect Jump to Z
    // ICALL: 1001 0100 0000 1111 (0x940F) - Indirect Call to Z
    m_instruction_table[0x940E] = &ATmegaInterpreter::exec_IJMP;
    m_instruction_table[0x940F] = &ATmegaInterpreter::exec_ICALL;
    m_instruction_table[0x950E] = &ATmegaInterpreter::exec_EIJMP;
    m_instruction_table[0x950F] = &ATmegaInterpreter::exec_EICALL;

    // Debug: verify JMP is set correctly
    spdlog::info("Instruction table[0x940C] set");

    // Debug log before ADDITIONAL INSTRUCTIONS
    spdlog::info("Building additional instructions...");

    // Single instructions
    m_instruction_table[0x9403] = &ATmegaInterpreter::exec_INC;
    m_instruction_table[0x9503] = &ATmegaInterpreter::exec_INC;
    m_instruction_table[0x940A] = &ATmegaInterpreter::exec_DEC;
    m_instruction_table[0x950A] = &ATmegaInterpreter::exec_DEC;
    m_instruction_table[0x9400] = &ATmegaInterpreter::exec_COM;
    m_instruction_table[0x9500] = &ATmegaInterpreter::exec_COM;
    m_instruction_table[0x9401] = &ATmegaInterpreter::exec_NEG;
    m_instruction_table[0x9501] = &ATmegaInterpreter::exec_NEG;
    m_instruction_table[0x920F] = &ATmegaInterpreter::exec_PUSH;
    m_instruction_table[0x930F] = &ATmegaInterpreter::exec_PUSH;
    m_instruction_table[0x900F] = &ATmegaInterpreter::exec_POP;
    m_instruction_table[0x910F] = &ATmegaInterpreter::exec_POP;

    // ========================================
    // ADDITIONAL AVR INSTRUCTIONS
    // Complete ATmega328P instruction set
    // ========================================

    // MUL: 1001 11rd dddd rrrr - Multiply (r25:r24 = Rd * Rr)
    spdlog::info("Building MUL instructions...");
    for (uint16_t op = 0x9C00; op <= 0x9FFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_MUL;
    }
    spdlog::info("MUL instructions done");

    // MULS: 0000 0010 dddd rrrr - Multiply Signed (r25:r24 = Rd * Rr)
    spdlog::info("Building MULS instructions...");
    for (uint16_t op = 0x0200; op <= 0x02FF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_MULS;
    }
    spdlog::info("MULS instructions done");

    // MULSU: 0000 0011 0ddd 0rrr - Multiply Signed with Unsigned
    for (uint16_t d = 16; d <= 23; d++) {
        for (uint16_t r = 16; r <= 23; r++) {
            uint16_t op = 0x0300 | ((d - 16) << 4) | (r - 16);
            m_instruction_table[op] = &ATmegaInterpreter::exec_MULSU;
        }
    }

    // FMUL: 0000 0011 0ddd 1rrr - Fractional Multiply Unsigned
    for (uint16_t d = 16; d <= 23; d++) {
        for (uint16_t r = 16; r <= 23; r++) {
            uint16_t op = 0x0380 | ((d - 16) << 4) | (r - 16);
            m_instruction_table[op] = &ATmegaInterpreter::exec_FMUL;
        }
    }

    // FMULS: 0000 0011 1ddd 0rrr - Fractional Multiply Signed
    for (uint16_t d = 16; d <= 23; d++) {
        for (uint16_t r = 16; r <= 23; r++) {
            uint16_t op = 0x03C0 | ((d - 16) << 4) | (r - 16);
            m_instruction_table[op] = &ATmegaInterpreter::exec_FMULS;
        }
    }

    // FMULSU: 0000 0011 1ddd 1rrr - Fractional Multiply Signed with Unsigned
    for (uint16_t d = 16; d <= 23; d++) {
        for (uint16_t r = 16; r <= 23; r++) {
            uint16_t op = 0x03E0 | ((d - 16) << 4) | (r - 16);
            m_instruction_table[op] = &ATmegaInterpreter::exec_FMULSU;
        }
    }

    // DES: 1001 0100 KKKK 1010 - DES (Data Encryption Step)
    for (uint16_t k = 0; k < 16; k++) {
        m_instruction_table[0x940A | (k << 4)] = &ATmegaInterpreter::exec_NOP_other; // DES - optional, treat as NOP
    }

    // BREAK: 1001 0101 1001 1000
    m_instruction_table[0x9598] = &ATmegaInterpreter::exec_NOP_other;

    // NOP: 0000 0000 0000 0000 (already set above)

    // SLEEP: 1001 0101 1000 1000
    m_instruction_table[0x9588] = &ATmegaInterpreter::exec_SLEEP;

    // WDR: 1001 0101 1010 1000 - Watchdog Reset
    m_instruction_table[0x95A8] = &ATmegaInterpreter::exec_WDR;

    // LPM: 1001 0101 1100 1000 (already set above as 0x95C8)

    // ELPM: 1001 0101 1101 1000
    m_instruction_table[0x95D8] = &ATmegaInterpreter::exec_ELPM;

    // SPM: 1001 0101 1110 1000 - Store Program Memory
    m_instruction_table[0x95E8] = &ATmegaInterpreter::exec_NOP_other; // SPM - flash write, treat as NOP

    // SPM Z+: 1001 0101 1111 1000
    m_instruction_table[0x95F8] = &ATmegaInterpreter::exec_NOP_other;

    // ESPM: 1001 0101 1111 1000
    m_instruction_table[0x95F8] = &ATmegaInterpreter::exec_NOP_other;

    // ========================================
    // BRANCH INSTRUCTIONS (Complete)
    // ========================================

    // BREQ: 1111 00kk kkkk k000 (skip if equal/Z=1)
    spdlog::info("Building BREQ instructions...");
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x000] = &ATmegaInterpreter::exec_BREQ;
        }
    }
    spdlog::info("BREQ instructions done");

    // BRNE: 1111 01kk kkkk k001 (skip if not equal/Z=0)
    spdlog::info("Building BRNE instructions...");
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x001] = &ATmegaInterpreter::exec_BRNE;
        }
    }
    spdlog::info("BRNE instructions done");

    // BRCS: 1111 01kk kkkk k000 (skip if carry set)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x000] = &ATmegaInterpreter::exec_BRCS;
        }
    }

    // BRCC: 1111 01kk kkkk k010 (skip if carry clear, same as BRSH)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x002] = &ATmegaInterpreter::exec_BRCC;
        }
    }

    // BRSH: 1111 01kk kkkk k010 (skip if same or higher/C=0)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x002] = &ATmegaInterpreter::exec_BRSH;
        }
    }

    // BRLO: 1111 00kk kkkk k010 (skip if lower/C=1)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x002] = &ATmegaInterpreter::exec_BRLO;
        }
    }

    // BRMI: 1111 01kk kkkk k010 (skip if minus/N=1)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x002] = &ATmegaInterpreter::exec_BRMI;
        }
    }

    // BRPL: 1111 01kk kkkk k011 (skip if plus/N=0)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x003] = &ATmegaInterpreter::exec_BRPL;
        }
    }

    // BRGE: 1111 01kk kkkk k100 (skip if greater or equal/S=0)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x004] = &ATmegaInterpreter::exec_BRGE;
        }
    }

    // BRLT: 1111 01kk kkkk k100 (skip if less than/S=1)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x004] = &ATmegaInterpreter::exec_BRLT;
        }
    }

    // BRHS: 1111 01kk kkkk k101 (skip if half carry set)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x005] = &ATmegaInterpreter::exec_BRHS;
        }
    }

    // BRHC: 1111 01kk kkkk k101 (skip if half carry clear)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x005] = &ATmegaInterpreter::exec_BRHC;
        }
    }

    // BRTS: 1111 01kk kkkk k110 (skip if T flag set)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x006] = &ATmegaInterpreter::exec_BRTS;
        }
    }

    // BRTC: 1111 01kk kkkk k110 (skip if T flag clear)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x006] = &ATmegaInterpreter::exec_BRTC;
        }
    }

    // BRVS: 1111 01kk kkkk k011 (skip if overflow set)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x003] = &ATmegaInterpreter::exec_BRVS;
        }
    }

    // BRVC: 1111 01kk kkkk k011 (skip if overflow clear)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x007] = &ATmegaInterpreter::exec_BRVC;
        }
    }

    // BRIE: 1111 01kk kkkk k111 (skip if interrupt enabled)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF400 | (i << 10) | (k << 3) | 0x007] = &ATmegaInterpreter::exec_BRIE;
        }
    }

    // BRID: 1111 01kk kkkk k111 (skip if interrupt disabled)
    for (int i = 0; i < 8; i++) {
        for (uint16_t k = 0; k < 128; k++) {
            m_instruction_table[0xF000 | (i << 10) | (k << 3) | 0x007] = &ATmegaInterpreter::exec_BRID;
        }
    }

    // ========================================
    // DATA TRANSFER INSTRUCTIONS
    // ========================================

    // LD: 10q0 qq0d dddd qqqq (Load Indirect using X/Y/Z)
    // LD X: 1001 000d dddd 1100 (0x900C), 1001 000d dddd 1101 (0x900D, X+), 1001 100d dddd 1100 (0x980C, -X)
    // LD Y: 1000 100d dddd 1000 (0x8808), 1001 000d dddd 1001 (0x9009, Y+), 1001 100d dddd 1010 (0x980A, -Y)
    // LD Z: 1000 000d dddd 0000 (0x8000), 1001 000d dddd 0001 (0x9001, Z+), 1001 100d dddd 1010 (0x980A, -Z)
    // Actually let me use the correct patterns from the AVR instruction set

    // LD X: 1001 000d dddd 1100 (0x900C to 0x91FC)
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9000 | (d << 4) | 0x000C] = &ATmegaInterpreter::exec_LD;
        m_instruction_table[0x9000 | (d << 4) | 0x000D] = &ATmegaInterpreter::exec_LD;  // LD X+
        m_instruction_table[0x9800 | (d << 4) | 0x000C] = &ATmegaInterpreter::exec_LD;  // LD -X
    }

    // LD Y: 1000 100d dddd 1000 (0x8088 to 81F8), 1001 000d dddd 1001 (0x9009), 1001 100d dddd 1010 (0x980A)
    spdlog::info("Building LD Y instructions...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x8000 | (d << 5) | 0x0008] = &ATmegaInterpreter::exec_LD;   // LD Y
        m_instruction_table[0x9000 | (d << 4) | 0x0009] = &ATmegaInterpreter::exec_LD;   // LD Y+
        m_instruction_table[0x9800 | (d << 4) | 0x000A] = &ATmegaInterpreter::exec_LD;   // LD -Y
    }
    spdlog::info("LD Y instructions done");

    // LD Z: 1000 000d dddd 0000 (0x8000 to 81F0), 1001 000d dddd 0001 (0x9001), 1001 100d dddd 1010 (0x980A)
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x8000 | (d << 3) | 0x0000] = &ATmegaInterpreter::exec_LD;   // LD Z
        m_instruction_table[0x9000 | (d << 4) | 0x0001] = &ATmegaInterpreter::exec_LD;   // LD Z+
        m_instruction_table[0x9800 | (d << 4) | 0x000A] = &ATmegaInterpreter::exec_LD;   // LD -Z
    }

    // ST: 10q0 qq0d dddd qqqq (Store Indirect using X/Y/Z)
    // ST X: 1001 001d dddd 1100 (0x920C), 1001 001d dddd 1101 (0x920D, X+), 1001 101d dddd 1100 (0x9A0C, -X)
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9200 | (d << 4) | 0x000C] = &ATmegaInterpreter::exec_ST;
        m_instruction_table[0x9200 | (d << 4) | 0x000D] = &ATmegaInterpreter::exec_ST;   // ST X+
        m_instruction_table[0x9A00 | (d << 4) | 0x000C] = &ATmegaInterpreter::exec_ST;   // ST -X
    }

    // ST Y: 1000 101d dddd 1000 (0x82C8), 1001 001d dddd 1001 (0x9209), 1001 101d dddd 1010 (0x9A0A)
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x8200 | (d << 5) | 0x0008] = &ATmegaInterpreter::exec_ST;   // ST Y
        m_instruction_table[0x9200 | (d << 4) | 0x0009] = &ATmegaInterpreter::exec_ST;   // ST Y+
        m_instruction_table[0x9A00 | (d << 4) | 0x000A] = &ATmegaInterpreter::exec_ST;   // ST -Y
    }

    // ST Z: 1000 001d dddd 0000 (0x8200), 1001 001d dddd 0001 (0x9201), 1001 101d dddd 1010 (0x9A0A)
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x8200 | (d << 3) | 0x0000] = &ATmegaInterpreter::exec_ST;   // ST Z
        m_instruction_table[0x9200 | (d << 4) | 0x0001] = &ATmegaInterpreter::exec_ST;   // ST Z+
        m_instruction_table[0x9A00 | (d << 4) | 0x000A] = &ATmegaInterpreter::exec_ST;   // ST -Z
    }

    // LDD Y+q: 10q0 qq1d dddd 1qqq
    spdlog::info("Building LDD Y+q part 1...");
    for (uint16_t q = 0; q < 32; q++) {
        for (uint16_t d = 0; d < 32; d++) {
            uint16_t op = 0x8000 | ((q & 0x20) << 8) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0008 | (d << 5);
            // This is complex, let's simplify
            // LDD Y+q: 1000 000d dddd 1qqq for q=0-7, 1000 001d dddd 0qqq for q=8-15, etc.
        }
    }
    spdlog::info("LDD Y+q part 1 done");

    // LDD Y+q: 10q0 qq0d dddd 1qqq where q is 0-63
    // This gives us: 10q0 qqqd dddd 1qqq format
    spdlog::info("Building LDD Y+q part 2...");
    for (uint16_t q = 0; q < 64; q++) {
        for (uint16_t d = 0; d < 32; d++) {
            // LDD Y+q
            uint16_t op = 0x8000 | ((q & 0x20) << 6) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0008 | (d << 5);
            m_instruction_table[0x8000 | ((q & 0x20) << 8) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0008 | (d << 4)] = &ATmegaInterpreter::exec_LD;
        }
    }
    spdlog::info("LDD Y+q part 2 done");

    // LDD Z+q: 10q0 qq0d dddd 0qqq where q is 0-63
    spdlog::info("Building LDD Z+q...");
    for (uint16_t q = 0; q < 64; q++) {
        for (uint16_t d = 0; d < 32; d++) {
            m_instruction_table[0x8000 | ((q & 0x20) << 8) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0000 | (d << 4)] = &ATmegaInterpreter::exec_LD;
        }
    }
    spdlog::info("LDD Z+q done");

    // STD Y+q: 10q0 qq1d dddd 1qqq
    spdlog::info("Building STD Y+q...");
    for (uint16_t q = 0; q < 64; q++) {
        for (uint16_t d = 0; d < 32; d++) {
            m_instruction_table[0x8200 | ((q & 0x20) << 8) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0008 | (d << 4)] = &ATmegaInterpreter::exec_ST;
        }
    }
    spdlog::info("STD Y+q done");

    // STD Z+q: 10q0 qq1d dddd 0qqq
    spdlog::info("Building STD Z+q...");
    for (uint16_t q = 0; q < 64; q++) {
        for (uint16_t d = 0; d < 32; d++) {
            m_instruction_table[0x8200 | ((q & 0x20) << 8) | ((q & 0x18) << 7) | ((q & 0x07) << 4) | 0x0000 | (d << 4)] = &ATmegaInterpreter::exec_ST;
        }
    }
    spdlog::info("STD Z+q done");

    // XCH: 1001 001d dddd 0100 (Exchange Z with Rd)
    spdlog::info("Building XCH...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9204 | (d << 4)] = &ATmegaInterpreter::exec_XCH;
    }
    spdlog::info("XCH done");

    // LAS: 1001 001d dddd 0101 (Load and Set from Z)
    spdlog::info("Building LAS...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9205 | (d << 4)] = &ATmegaInterpreter::exec_LAS;
    }
    spdlog::info("LAS done");

    // LAC: 1001 001d dddd 0110 (Load and Clear from Z)
    spdlog::info("Building LAC...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9206 | (d << 4)] = &ATmegaInterpreter::exec_LAC;
    }
    spdlog::info("LAC done");

    // LAT: 1001 001d dddd 0111 (Load and Toggle from Z)
    spdlog::info("Building LAT...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9207 | (d << 4)] = &ATmegaInterpreter::exec_LAT;
    }
    spdlog::info("LAT done");

    // ========================================
    // BIT AND BIT-TEST INSTRUCTIONS
    // ========================================

    // LSR: 1001 010d dddd 0110 (Logical Shift Right)
    spdlog::info("Building LSR...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9406 | (d << 4)] = &ATmegaInterpreter::exec_LSR;
    }
    spdlog::info("LSR done");

    // ROR: 1001 010d dddd 0111 (Rotate Right)
    spdlog::info("Building ROR...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9407 | (d << 4)] = &ATmegaInterpreter::exec_ROR;
    }
    spdlog::info("ROR done");

    // ASR: 1001 010d dddd 0101 (Arithmetic Shift Right)
    spdlog::info("Building ASR...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9405 | (d << 4)] = &ATmegaInterpreter::exec_ASR;
    }
    spdlog::info("ASR done");

    // SWAP: 1001 010d dddd 0010 (Swap nibbles)
    spdlog::info("Building SWAP...");
    for (uint16_t d = 0; d < 32; d++) {
        m_instruction_table[0x9402 | (d << 4)] = &ATmegaInterpreter::exec_SWAP;
    }
    spdlog::info("SWAP done");

    // SBI: 1001 1010 AAAA Abbb (Set Bit in I/O Register)
    spdlog::info("Building SBI...");
    for (uint16_t op = 0x9A00; op <= 0x9AFF; op++) {
        if ((op & 0x0F00) <= 0x0700 && (op & 0x000F) <= 0x0007) {
            m_instruction_table[op] = &ATmegaInterpreter::exec_SBI;
        }
    }
    spdlog::info("SBI done");

    // CBI: 1001 1000 AAAA Abbb (Clear Bit in I/O Register)
    spdlog::info("Building CBI...");
    for (uint16_t op = 0x9800; op <= 0x98FF; op++) {
        if ((op & 0x0F00) <= 0x0700 && (op & 0x000F) <= 0x0007) {
            m_instruction_table[op] = &ATmegaInterpreter::exec_CBI;
        }
    }
    spdlog::info("CBI done");

    // SBIC: 1001 1001 AAAA Abbb (Skip if Bit in I/O Register is Clear)
    spdlog::info("Building SBIC...");
    for (uint16_t op = 0x9900; op <= 0x99FF; op++) {
        if ((op & 0x0F00) <= 0x0700 && (op & 0x000F) <= 0x0007) {
            m_instruction_table[op] = &ATmegaInterpreter::exec_SBIC;
        }
    }
    spdlog::info("SBIC done");

    // SBIS: 1001 1011 AAAA Abbb (Skip if Bit in I/O Register is Set)
    spdlog::info("Building SBIS...");
    for (uint16_t op = 0x9B00; op <= 0x9BFF; op++) {
        if ((op & 0x0F00) <= 0x0700 && (op & 0x000F) <= 0x0007) {
            m_instruction_table[op] = &ATmegaInterpreter::exec_SBIS;
        }
    }
    spdlog::info("SBIS done");

    // SBRC: 1111 110r rrrr 0bbb (Skip if Bit in Register is Clear)
    // Range: 0xF800 to 0xFBFF (1024 opcodes)
    spdlog::info("Building SBRC...");
    for (uint16_t op = 0xF800; op < 0xFC00; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_SBRC;
    }
    spdlog::info("SBRC done");

    // SBRS: 1111 111r rrrr 0bbb (Skip if Bit in Register is Set)
    // Range: 0xFC00 to 0xFFFF (1024 opcodes)
    spdlog::info("Building SBRS...");
    for (int op = 0xFC00; op <= 0xFFFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_SBRS;
    }
    spdlog::info("SBRS done");

    // BSET: 1001 0100 0bbb 1000 (Set Bit in SREG) - b is 0-7 (T, H, S, V, N, Z, C, I)
    // Actually: 1001 0100 1111 1000 (BSET S), 1001 0100 1110 1000 (BSET T), etc.
    // CLI: 1001 0100 1111 1000, SEI: 1001 0100 0111 1000
    m_instruction_table[0x9408] = &ATmegaInterpreter::exec_SEC;  // SEC (Set C)
    m_instruction_table[0x9418] = &ATmegaInterpreter::exec_SEZ;  // SEZ (Set Z)
    m_instruction_table[0x9428] = &ATmegaInterpreter::exec_SEN;  // SEN (Set N)
    m_instruction_table[0x9438] = &ATmegaInterpreter::exec_SEV;  // SEV (Set V)
    m_instruction_table[0x9448] = &ATmegaInterpreter::exec_SES;  // SES (Set S)
    m_instruction_table[0x9458] = &ATmegaInterpreter::exec_SEH;  // SEH (Set H)
    m_instruction_table[0x9468] = &ATmegaInterpreter::exec_SET;  // SET (Set T)
    m_instruction_table[0x9478] = &ATmegaInterpreter::exec_SEI;  // SEI (Set I)

    // BCLR: 1001 0100 1bbb 1000 (Clear Bit in SREG)
    m_instruction_table[0x9488] = &ATmegaInterpreter::exec_CLC;  // CLC (Clear C)
    m_instruction_table[0x9498] = &ATmegaInterpreter::exec_CLZ;  // CLZ (Clear Z)
    m_instruction_table[0x94A8] = &ATmegaInterpreter::exec_CLN;  // CLN (Clear N)
    m_instruction_table[0x94B8] = &ATmegaInterpreter::exec_CLV;  // CLV (Clear V)
    m_instruction_table[0x94C8] = &ATmegaInterpreter::exec_CLS;  // CLS (Clear S)
    m_instruction_table[0x94D8] = &ATmegaInterpreter::exec_CLH;  // CLH (Clear H)
    m_instruction_table[0x94E8] = &ATmegaInterpreter::exec_CLT;  // CLT (Clear T)
    m_instruction_table[0x94F8] = &ATmegaInterpreter::exec_CLI;  // CLI (Clear I)

    // BST: 1001 010d dddd 0110 (Bit Store from Register to T)
    for (uint16_t d = 0; d < 32; d++) {
        for (uint16_t b = 0; b < 8; b++) {
            m_instruction_table[0xF800 | (d << 4) | (b)] = &ATmegaInterpreter::exec_BST;
        }
    }

    // BLD: 1001 010d dddd 0111 (Bit Load from T to Register)
    for (uint16_t d = 0; d < 32; d++) {
        for (uint16_t b = 0; b < 8; b++) {
            m_instruction_table[0xF800 | (d << 4) | (b)] = &ATmegaInterpreter::exec_BLD;
        }
    }

    // SER: 1110 1111 0ddd 1111 (Set all bits in Register, same as LDI Rd, 0xFF)
    // d is 0-15 (representing R16-R31), encoded as dddd in bits 7:4
    for (uint16_t d = 0; d < 16; d++) {
        m_instruction_table[0xEF00 | (d << 4) | 0x000F] = &ATmegaInterpreter::exec_LDI;  // LDI with 0xFF
    }

    // ========================================
    // LOGICAL INSTRUCTIONS (already set above)
    // AND: 0x2000-0x23FF, OR: 0x2800-0x2BFF, EOR: 0x2400-0x27FF
    // ========================================

    // ========================================
    // ARITHMETIC INSTRUCTIONS
    // ========================================

    // ADD: 0000 11rd dddd rrrr (0x0C00 to 0x0FFF) - already set above

    // ADC: 0001 11rd dddd rrrr (0x1C00 to 0x1FFF) - already set above

    // SUB: 0001 10rd dddd rrrr (0x1800 to 0x1BFF) - already set above

    // SUBI: 0101 KKKK dddd KKKK (0x5000 to 0x5FF0) - already set above

    // SBCI: 0100 KKKK dddd KKKK (0x4000 to 0x4FF0) - already set above

    // SBC: 0000 10rd dddd rrrr (0x0800 to 0x0BFF)
    for (uint16_t op = 0x0800; op <= 0x0BFF; op++) {
        m_instruction_table[op] = &ATmegaInterpreter::exec_SBC;
    }

    // ANDI: 0111 KKKK dddd KKKK (0x7000 to 0x7FF0) - already set above

    // ORI: 0110 KKKK dddd KKKK (0x6000 to 0x6FF0) - already set above

    // CPI: 0011 KKKK dddd KKKK (0x3000 to 0x3FF0) - already set above

    // CP: 0001 01rd dddd rrrr (0x1400 to 0x17FF) - already set above

    // CPC: 0000 01rd dddd rrrr (0x0400 to 0x07FF) - already set above

    // ========================================
    // JUMP AND CALL INSTRUCTIONS
    // ========================================

    // RJMP: 1100 kkkk kkkk kkkk (0xC000 to 0xCFFF) - already set above

    // RCALL: 1101 kkkk kkkk kkkk (0xD000 to 0xDFFF) - already set above

    // JMP: 1001 010k kkkk 110k / 1001 010k kkkk 111k - already set above

    // CALL: 1001 010k kkkk 111k (k varies, but NOT 0x940E/0x940F/0x950E/0x950F)
    // CALL first word: bits 12-15 = 1001, bit 11 = 0, bit 10 = 1, bits 3-0 = 1110 (0xE) or 1111 (0xF)
    // But 0x940E=IJMP, 0x940F=ICALL, 0x950E=EIJMP, 0x950F=EICALL are special cases
    for (uint16_t h = 0x94; h <= 0x95; h++) {
        for (uint16_t k = 0; k < 4; k++) {
            uint16_t op = (h << 8) | (k << 4) | 0x000E;
            if (op != 0x940E && op != 0x950E) {  // Skip IJMP and EIJMP
                m_instruction_table[op] = &ATmegaInterpreter::exec_CALL;
            }
        }
    }

    // IJMP: 1001 0100 0000 1110 - already set above
    // ICALL: 1001 0100 0000 1111 - already set above
    // EIJMP: 1001 0101 0000 1110 - already set above
    // EICALL: 1001 0101 0000 1111 - already set above

    // RET: 1001 0101 0000 1000 - already set above

    // RETI: 1001 0101 0001 1000 - already set above

    // ========================================
    // DATA DIRECT INSTRUCTIONS
    // ========================================

    // LDS: 1001 000d dddd 0000 (2-word) - already set above

    // STS: 1001 001d dddd 0000 (2-word) - already set above

    // ========================================
    // IO INSTRUCTIONS
    // ========================================

    // IN: 1011 0AAd dddd AAAA - already set above

    // OUT: 1011 1AAd dddd AAAA - already set above

    // ========================================
    // PUSH/POP INSTRUCTIONS
    // ========================================

    // PUSH: 1001 001d dddd 1111
    for (uint16_t d = 0; d < 32; d++) {
        uint16_t op = 0x9200 | (d << 4) | 0x000F;
        m_instruction_table[op] = &ATmegaInterpreter::exec_PUSH;
        op = 0x9300 | (d << 4) | 0x000F;
        m_instruction_table[op] = &ATmegaInterpreter::exec_PUSH;
    }

    // POP: 1001 000d dddd 1111
    for (uint16_t d = 0; d < 32; d++) {
        uint16_t op = 0x9000 | (d << 4) | 0x000F;
        m_instruction_table[op] = &ATmegaInterpreter::exec_POP;
        op = 0x9100 | (d << 4) | 0x000F;
        m_instruction_table[op] = &ATmegaInterpreter::exec_POP;
    }

    // ========================================
    // ROTATE THROUGH CARRY INSTRUCTIONS
    // ========================================

    // LSL: 0000 11rd dddd rrrr where rrrr = dddd + 0 (same as ADD Rd, Rd)
    // Actually LSL is encoded as ADD Rd, Rd
    // So it's already covered by the ADD instruction range

    // LSR: 1001 010d dddd 0110 - already set above

    // ROL: 0000 11rd dddd rrrr where rrrr = dddd + 0 (same as ADC Rd, Rd with C=0)
    // Actually ROL is encoded as ADC Rd, Rd with C=0
    // So it's already covered by the ADC instruction range

    // ROR: 1001 010d dddd 0111 - already set above

    // ASR: 1001 010d dddd 0101 - already set above

    // ========================================
    // MISCELLANEOUS INSTRUCTIONS
    // ========================================

    // MOVW: 0000 0001 dddd rrrr (Copy Register Word, d and r are even)
    spdlog::info("Building MOVW...");
    for (uint16_t d = 0; d < 32; d += 2) {
        for (uint16_t r = 0; r < 32; r += 2) {
            uint16_t op = 0x0100 | ((d >> 1) << 4) | (r >> 1);
            m_instruction_table[op] = &ATmegaInterpreter::exec_MOVW;
        }
    }
    spdlog::info("MOVW done");

    // ADIW: 1001 0110 KKdd KKKK (Add Immediate to Word)
    // Valid for d=24,26,28,30 (X,Y,Z)
    spdlog::info("Building ADIW...");
    for (uint16_t q = 0; q < 4; q++) {
        for (uint16_t k = 0; k < 64; k++) {
            uint16_t op = 0x9600 | (q << 4) | ((k & 0x30) << 2) | (k & 0x0F);
            m_instruction_table[op] = &ATmegaInterpreter::exec_ADIW;
        }
    }
    spdlog::info("ADIW done");

    // SBIW: 1001 0111 KKdd KKKK (Subtract Immediate from Word)
    spdlog::info("Building SBIW...");
    for (uint16_t q = 0; q < 4; q++) {
        for (uint16_t k = 0; k < 64; k++) {
            uint16_t op = 0x9700 | (q << 4) | ((k & 0x30) << 2) | (k & 0x0F);
            m_instruction_table[op] = &ATmegaInterpreter::exec_SBIW;
        }
    }
    spdlog::info("SBIW done");

    // CBI: 1001 1000 AAAA Abbb - already set above

    // SBIC: 1001 1001 AAAA Abbb - already set above

    // SBI: 1001 1010 AAAA Abbb - already set above

    // SBIS: 1001 1011 AAAA Abbb - already set above

    // ========================================
    // COMPLETE BRANCH INSTRUCTIONS
    // ========================================

    // BRBC: 1111 01kk kkkk kbbb (Branch if Bit in SREG is Clear)
    // b=0: BRCS (same as BRLO), b=1: BREQ (same as BRIE for I bit), etc.
    spdlog::info("Building BRBC...");
    for (int b = 0; b < 8; b++) {
        for (int k = 0; k < 64; k++) {
            m_instruction_table[0xF400 | (k << 3) | b] = &ATmegaInterpreter::exec_BRBC;
        }
    }
    spdlog::info("BRBC done");

    // BRBS: 1111 00kk kkkk kbbb (Branch if Bit in SREG is Set)
    spdlog::info("Building BRBS...");
    for (int b = 0; b < 8; b++) {
        for (int k = 0; k < 64; k++) {
            m_instruction_table[0xF000 | (k << 3) | b] = &ATmegaInterpreter::exec_BRBS;
        }
    }
    spdlog::info("BRBS done");
    spdlog::info("Instruction table build complete!");
}

bool ATmegaInterpreter::load_firmware(const std::string& path) {
    spdlog::info("Loading firmware into interpreter: {}", path);

    FirmwareLoader loader;
    if (!loader.load(path)) {
        spdlog::error("Failed to load firmware: {}", loader.error());
        return false;
    }

    // Clear flash
    std::fill(m_flash.begin(), m_flash.end(), 0xFFFF);

    // Load binary into flash using memory map (preserves addresses)
    const auto& memory_map = loader.get_memory_map();

    size_t total_bytes = 0;
    for (const auto& [addr, data] : memory_map) {
        // Convert byte address to word address and ensure it's within flash bounds
        uint32_t word_addr = addr / 2;

        // Convert bytes to words (little endian for AVR) and place at correct address
        for (size_t i = 0; i < data.size(); i += 2) {
            uint32_t flash_idx = word_addr + (i / 2);
            if (flash_idx < m_flash.size()) {
                uint16_t word = data[i];
                if (i + 1 < data.size()) {
                    word |= data[i + 1] << 8;
                }
                m_flash[flash_idx] = word;
            }
        }
        total_bytes += data.size();
    }

    reset();

    spdlog::info("Firmware loaded: {} bytes, {} words, {} segments",
                 total_bytes, total_bytes / 2, memory_map.size());
    return true;
}

void ATmegaInterpreter::reset() {
    m_state.PC = 0x0000;  // Reset vector
    m_state.SP = 0xFFFF;  // Stack starts at top of SRAM
    m_state.SREG = {};
    m_state.cycles = 0;
    m_instruction_count = 0;

    spdlog::debug("Interpreter reset: PC=0x{:04X}, SP=0x{:04X}",
                  m_state.PC, m_state.SP);
}

bool ATmegaInterpreter::step() {
    if (!is_loaded()) {
        return false;
    }

    // Check for breakpoint
    if (has_breakpoint(m_state.PC)) {
        spdlog::info("Breakpoint hit at 0x{:04X}", m_state.PC);
    }

    Instruction inst = decode_instruction(m_state.PC);

    // Execute instruction BEFORE incrementing PC
    // This is important for multi-word instructions like JMP/CALL
    // that need to read the subsequent words
    bool continue_exec = execute_instruction(inst);

    // Now increment PC (may have been modified by branches/calls)
    // For normal instructions, we add inst.size
    // For JMP/CALL, the handler already set PC to the target
    // We need to NOT increment if the instruction was a jump/call

    // Check if this was a jump/call instruction
    uint16_t opcode = inst.opcode;
    bool is_jump = ((opcode & 0xFE0F) == 0x940C || (opcode & 0xFE0F) == 0x940D ||
                    (opcode & 0xFE0F) == 0x950C || (opcode & 0xFE0F) == 0x950D ||
                    opcode == 0x940E || opcode == 0x950E ||  // IJMP, EIJMP
                    opcode == 0x940F || opcode == 0x950F);  // ICALL, EICALL

    if (!is_jump && (opcode & 0xF000) != 0xC000 && (opcode & 0xF000) != 0xD000) {
        // Not JMP/CALL/RCALL/RJMP - increment PC normally
        m_state.PC += inst.size;
    }
    // For JMP/CALL/RJMP/RCALL, PC is already set by the instruction handler

    m_state.cycles += inst.cycles;
    m_instruction_count++;

    return continue_exec;
}

bool ATmegaInterpreter::step_instructions(uint32_t count) {
    for (uint32_t i = 0; i < count && is_loaded(); i++) {
        if (!step()) {
            return false;
        }
    }
    return true;
}

void ATmegaInterpreter::execute_for_us(uint32_t duration_us) {
    uint64_t target_cycles = (uint64_t)duration_us * (F_CPU / 1000000);
    uint64_t start_cycles = m_state.cycles;

    while ((m_state.cycles - start_cycles) < target_cycles && is_loaded()) {
        if (!step()) {
            break;
        }
    }
}

ATmegaInterpreter::Instruction ATmegaInterpreter::decode_instruction(uint16_t addr) const {
    if (addr >= m_flash.size()) {
        return {0xFFFF, 0, "INVALID", 1, 1};
    }

    uint16_t opcode = m_flash[addr];
    uint16_t operands = (addr + 1 < m_flash.size()) ? m_flash[addr + 1] : 0;

    Instruction inst;
    inst.opcode = opcode;
    inst.operands = operands;
    inst.size = 1;
    inst.cycles = 1;

    // Decode based on opcode pattern
    // Check instruction table first (for debugging/disassembly)
    ExecFunc handler = m_instruction_table[opcode];
    if (handler == &ATmegaInterpreter::exec_NOP_other) {
        // Check if this is actually a NOP
        if (opcode != 0x0000) {
            inst.mnemonic = "UNKNOWN";
        } else {
            inst.mnemonic = "NOP";
        }
    } else {
        inst.mnemonic = "?";  // Will be filled by execution
    }

    // Determine instruction size and cycles
    if ((opcode & 0xFE0E) == 0x9200 || (opcode & 0xFE0E) == 0x9000) {
        // STS/LDS - 2 words
        inst.size = 2;
        inst.cycles = 2;
    } else if ((opcode & 0xFE0F) == 0x940C || (opcode & 0xFE0F) == 0x940D ||
               (opcode & 0xFE0F) == 0x950C || (opcode & 0xFE0F) == 0x950D ||
               (opcode & 0xFE0F) == 0x940E || (opcode & 0xFE0F) == 0x940F ||
               (opcode & 0xFE0F) == 0x950E || (opcode & 0xFE0F) == 0x950F) {
        // JMP/CALL - 2 words
        inst.size = 2;
        inst.cycles = 3;  // JMP takes 3 cycles
    } else if ((opcode & 0xFE0F) == 0x9400) {
        // Single-word instructions (non-JMP/CALL)
        inst.cycles = 1;
    } else if ((opcode & 0xF000) == 0xC000 || (opcode & 0xF000) == 0xD000) {
        // RJMP/RCALL
        inst.size = 1;
        inst.cycles = 2;
    } else if ((opcode & 0xFC00) == 0xF000 || (opcode & 0xFC00) == 0xF800 ||
               (opcode & 0xFE00) == 0xF000) {
        // Branch instructions
        inst.size = 1;
        inst.cycles = 1;  // 2 if taken
    }

    return inst;
}

bool ATmegaInterpreter::execute_instruction(const Instruction& inst) {
    uint16_t opcode = inst.opcode;

    // Look up handler in instruction table
    ExecFunc handler = m_instruction_table[opcode];

    // Execute the instruction
    return (this->*handler)(inst);
}

// ===== Instruction Implementations =====

bool ATmegaInterpreter::exec_NOP(const Instruction& inst) {
    (void)inst;
    return true;
}

bool ATmegaInterpreter::exec_MOV(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    m_mcu.memory().gp_registers[rd] = m_mcu.memory().gp_registers[rr];
    return true;
}

bool ATmegaInterpreter::exec_LDI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    // LDI works on registers 16-31 (r16-r31)
    m_mcu.memory().gp_registers[16 + rd] = k;
    return true;
}

bool ATmegaInterpreter::exec_STS(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint16_t addr = inst.operands;  // 16-bit address

    m_mcu.memory().sram[addr] = m_mcu.memory().gp_registers[rd];
    return true;
}

bool ATmegaInterpreter::exec_LDS(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint16_t addr = inst.operands;  // 16-bit address

    m_mcu.memory().gp_registers[rd] = m_mcu.memory().sram[addr];
    return true;
}

bool ATmegaInterpreter::exec_ST(const Instruction& inst) {
    // Store using indirect addressing (X/Y/Z pointers)
    // Simplified - just log
    spdlog::trace("ST instruction (not fully implemented)");
    return true;
}

bool ATmegaInterpreter::exec_LD(const Instruction& inst) {
    // Load using indirect addressing
    spdlog::trace("LD instruction (not fully implemented)");
    return true;
}

bool ATmegaInterpreter::exec_PUSH(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_mcu.memory().sram[m_state.SP] = m_mcu.memory().gp_registers[rd];
    m_state.SP--;
    return true;
}

bool ATmegaInterpreter::exec_POP(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_state.SP++;
    m_mcu.memory().gp_registers[rd] = m_mcu.memory().sram[m_state.SP];
    return true;
}

bool ATmegaInterpreter::exec_IN(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t addr = get_io_addr(inst.opcode);

    m_mcu.memory().gp_registers[rd] = m_mcu.memory().read_io(addr);
    return true;
}

bool ATmegaInterpreter::exec_OUT(const Instruction& inst) {
    uint8_t rr = get_rr(inst.opcode);
    uint8_t addr = get_io_addr(inst.opcode);

    m_mcu.memory().write_io(addr, m_mcu.memory().gp_registers[rr]);
    return true;
}

bool ATmegaInterpreter::exec_ANDI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    uint8_t result = m_mcu.memory().gp_registers[16 + rd] & k;
    m_mcu.memory().gp_registers[16 + rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_ORI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    uint8_t result = m_mcu.memory().gp_registers[16 + rd] | k;
    m_mcu.memory().gp_registers[16 + rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_EOR(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t result = m_mcu.memory().gp_registers[rd] ^ m_mcu.memory().gp_registers[rr];
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_AND(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t result = m_mcu.memory().gp_registers[rd] & m_mcu.memory().gp_registers[rr];
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_OR(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t result = m_mcu.memory().gp_registers[rd] | m_mcu.memory().gp_registers[rr];
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_COM(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_mcu.memory().gp_registers[rd] = ~m_mcu.memory().gp_registers[rd];
    uint8_t result = m_mcu.memory().gp_registers[rd];

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(0, 0, result, false);
    update_c_flag(0, 0, false);
    m_state.SREG.C = true;  // COM sets C flag
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_NEG(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_mcu.memory().gp_registers[rd] = -m_mcu.memory().gp_registers[rd];
    uint8_t result = m_mcu.memory().gp_registers[rd];

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(0, m_mcu.memory().gp_registers[rd], true);
    update_v_flag(0, m_mcu.memory().gp_registers[rd], result, true);
    update_h_flag(0, m_mcu.memory().gp_registers[rd], true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_INC(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_mcu.memory().gp_registers[rd]++;
    uint8_t result = m_mcu.memory().gp_registers[rd];

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(result - 1, 1, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_DEC(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);

    m_mcu.memory().gp_registers[rd]--;
    uint8_t result = m_mcu.memory().gp_registers[rd];

    update_z_flag(result);
    update_n_flag(result);
    update_v_flag(result + 1, 0xFF, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_ADD(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint16_t result = a + b;

    m_mcu.memory().gp_registers[rd] = (uint8_t)result;

    update_z_flag((uint8_t)result);
    update_n_flag((uint8_t)result);
    update_c_flag(a, b, false);
    update_h_flag(a, b, false);
    update_v_flag(a, b, (uint8_t)result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_ADC(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint8_t c = m_state.SREG.C ? 1 : 0;
    uint16_t result = a + b + c;

    m_mcu.memory().gp_registers[rd] = (uint8_t)result;

    update_z_flag((uint8_t)result);
    update_n_flag((uint8_t)result);
    update_c_flag(a, b + c, false);
    update_h_flag(a, b + c, false);
    update_v_flag(a, b + c, (uint8_t)result, false);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_SUB(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint8_t result = a - b;

    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, b, true);
    update_h_flag(a, b, true);
    update_v_flag(a, b, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_SBC(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint8_t c = m_state.SREG.C ? 1 : 0;
    uint8_t result = a - b - c;

    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, b + c, true);
    update_h_flag(a, b + c, true);
    update_v_flag(a, b + c, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_SUBI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[16 + rd];
    uint8_t result = a - k;

    m_mcu.memory().gp_registers[16 + rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, k, true);
    update_h_flag(a, k, true);
    update_v_flag(a, k, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_SBCI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[16 + rd];
    uint8_t c = m_state.SREG.C ? 1 : 0;
    uint8_t result = a - k - c;

    m_mcu.memory().gp_registers[16 + rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, k + c, true);
    update_h_flag(a, k + c, true);
    update_v_flag(a, k + c, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_RJMP(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    // Relative jump: PC = PC + offset + 1
    m_state.PC = m_state.PC + offset;

    return true;
}

bool ATmegaInterpreter::exec_RCALL(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    // Push return address
    uint16_t ret_addr = m_state.PC;
    m_mcu.memory().sram[m_state.SP] = ret_addr & 0xFF;
    m_state.SP--;
    m_mcu.memory().sram[m_state.SP] = (ret_addr >> 8) & 0xFF;
    m_state.SP--;

    // Relative call
    m_state.PC = m_state.PC + offset;

    return true;
}

bool ATmegaInterpreter::exec_RET(const Instruction& inst) {
    (void)inst;

    // Pop return address
    m_state.SP++;
    uint8_t ret_high = m_mcu.memory().sram[m_state.SP];
    m_state.SP++;
    uint8_t ret_low = m_mcu.memory().sram[m_state.SP];

    m_state.PC = (ret_high << 8) | ret_low;

    return true;
}

bool ATmegaInterpreter::exec_RETI(const Instruction& inst) {
    exec_RET(inst);
    m_state.SREG.I = true;  // Enable interrupts
    return true;
}

bool ATmegaInterpreter::exec_CALL(const Instruction& inst) {
    (void)inst;
    // Full CALL implementation (3 words)
    // Simplified for now
    return true;
}

bool ATmegaInterpreter::exec_JMP(const Instruction& inst) {
    // JMP is a 2-word instruction (4 bytes total)
    // Encoding: 1001 010k kkkk 110k / kkkk kkkk kkkk kkkk
    //
    // For ATmega328P and similar devices (< 128KB flash):
    //   Only lower 16 bits of address are used (PC is 16-bit word address)
    //   The JMP instruction effectively becomes: addr = second_word
    //
    // The first word format is: 1001 0100 0000 1100 (0x940C) for 16-bit jumps
    // where the upper address bits are all 0
    //
    // For devices with larger flash, the encoding would be:
    //   addr[15:0] = second_word
    //   addr[21:16] = bits from first_word

    uint16_t second_word = m_flash[m_state.PC + 1];  // Get second word (lower 16 bits of addr)

    // For ATmega328P (16-bit PC), just use the second word as destination
    uint32_t addr = second_word;

    // JMP destination is a word address
    // Note: The step() function will add inst.size (2) after this,
    // so we need to account for that OR we set PC directly and let it be.
    // Actually, for jumps, we want to override the normal PC increment.
    // The current flow sets PC here, then step() adds inst.size.
    // Solution: set PC to target address and DON'T add inst.size in step() for jumps.
    // But step() doesn't know it's a jump...
    //
    // Workaround: Since step() will add inst.size (2) after we return,
    // we should set PC to (addr - inst.size) so that after adding 2, we get addr.
    // But actually, for JMP, the target is absolute, so we should just set it directly.

    // The issue is that step() always does PC += inst.size after execute_instruction()
    // For JMP/CALL, this is wrong because we set PC to the absolute target.
    // We need to either:
    // 1. Have execute_instruction return a flag to skip PC increment
    // 2. Have JMP set PC to (target - inst.size) as a workaround

    // Since we now execute BEFORE incrementing PC in step(),
    // we can set PC directly to the target address
    m_state.PC = addr;

    return true;
}

bool ATmegaInterpreter::exec_IJMP(const Instruction& inst) {
    (void)inst;
    // IJMP - Indirect Jump to Z
    // Jump to address in Z register (R31:R30)
    uint8_t z_low = m_mcu.memory().gp_registers[30];  // R30
    uint8_t z_high = m_mcu.memory().gp_registers[31]; // R31
    uint16_t z = (z_high << 8) | z_low;

    // Z contains word address
    m_state.PC = z;

    return true;
}

bool ATmegaInterpreter::exec_ICALL(const Instruction& inst) {
    (void)inst;
    // ICALL - Indirect Call to Z
    // Push return address, then jump to address in Z register

    // Push return address (current PC + 1 for the instruction after this)
    uint16_t ret_addr = m_state.PC + 1;
    m_mcu.memory().sram[m_state.SP] = ret_addr & 0xFF;
    m_state.SP--;
    m_mcu.memory().sram[m_state.SP] = (ret_addr >> 8) & 0xFF;
    m_state.SP--;

    // Jump to Z
    uint8_t z_low = m_mcu.memory().gp_registers[30];  // R30
    uint8_t z_high = m_mcu.memory().gp_registers[31]; // R31
    uint16_t z = (z_high << 8) | z_low;

    m_state.PC = z;

    return true;
}

bool ATmegaInterpreter::exec_EIJMP(const Instruction& inst) {
    (void)inst;
    // EIJMP - Extended Indirect Jump (for devices with > 128KB flash)
    // Uses Z register and EIND register
    // For ATmega328P (32KB), this behaves like IJMP
    uint8_t z_low = m_mcu.memory().gp_registers[30];
    uint8_t z_high = m_mcu.memory().gp_registers[31];
    uint16_t z = (z_high << 8) | z_low;

    m_state.PC = z;

    return true;
}

bool ATmegaInterpreter::exec_EICALL(const Instruction& inst) {
    (void)inst;
    // EICALL - Extended Indirect Call
    // For ATmega328P, behaves like ICALL
    uint16_t ret_addr = m_state.PC + 1;
    m_mcu.memory().sram[m_state.SP] = ret_addr & 0xFF;
    m_state.SP--;
    m_mcu.memory().sram[m_state.SP] = (ret_addr >> 8) & 0xFF;
    m_state.SP--;

    uint8_t z_low = m_mcu.memory().gp_registers[30];
    uint8_t z_high = m_mcu.memory().gp_registers[31];
    uint16_t z = (z_high << 8) | z_low;

    m_state.PC = z;

    return true;
}

bool ATmegaInterpreter::exec_CP(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint8_t result = a - b;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, b, true);
    update_h_flag(a, b, true);
    update_v_flag(a, b, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_CPC(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = get_rr(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[rd];
    uint8_t b = m_mcu.memory().gp_registers[rr];
    uint8_t c = m_state.SREG.C ? 1 : 0;
    uint8_t result = a - b - c;

    // Only update flags (don't store result)
    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, b + c, true);
    update_h_flag(a, b + c, true);
    update_v_flag(a, b + c, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_CPI(const Instruction& inst) {
    uint8_t rd = get_rd(inst.opcode);
    uint8_t k = get_k(inst.opcode);

    uint8_t a = m_mcu.memory().gp_registers[16 + rd];
    uint8_t result = a - k;

    update_z_flag(result);
    update_n_flag(result);
    update_c_flag(a, k, true);
    update_h_flag(a, k, true);
    update_v_flag(a, k, result, true);
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_BREQ(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (m_state.SREG.Z) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_BRNE(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (!m_state.SREG.Z) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_BRSH(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (!m_state.SREG.C) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_BRLO(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (m_state.SREG.C) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_BRMI(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (m_state.SREG.N) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_BRPL(const Instruction& inst) {
    int8_t offset = (int8_t)(inst.opcode & 0xFF);

    if (!m_state.SREG.N) {
        m_state.PC = m_state.PC + offset;
    }

    return true;
}

bool ATmegaInterpreter::exec_SBRS(const Instruction& inst) {
    // SBRS: Skip if Bit in Register is Set
    uint8_t rr = (inst.opcode >> 4) & 0x1F;
    uint8_t bit = inst.opcode & 0x07;

    if (m_mcu.memory().gp_registers[rr] & (1 << bit)) {
        // Skip next instruction (add 1 to PC to skip)
        m_state.PC += 1;
    }
    return true;
}

bool ATmegaInterpreter::exec_SBRC(const Instruction& inst) {
    // SBRC: Skip if Bit in Register is Clear
    uint8_t rr = (inst.opcode >> 4) & 0x1F;
    uint8_t bit = inst.opcode & 0x07;

    if (!(m_mcu.memory().gp_registers[rr] & (1 << bit))) {
        // Skip next instruction
        m_state.PC += 1;
    }
    return true;
}

bool ATmegaInterpreter::exec_SBIC(const Instruction& inst) {
    // SBIC: Skip if Bit in I/O Register is Clear
    uint8_t addr = ((inst.opcode >> 3) & 0x18) | (inst.opcode & 0x07);
    uint8_t bit = (inst.opcode >> 4) & 0x07;

    uint8_t io_val = m_mcu.memory().read_io(addr);
    if (!(io_val & (1 << bit))) {
        // Skip next instruction
        m_state.PC += 1;
    }
    return true;
}

bool ATmegaInterpreter::exec_SBIS(const Instruction& inst) {
    // SBIS: Skip if Bit in I/O Register is Set
    uint8_t addr = ((inst.opcode >> 3) & 0x18) | (inst.opcode & 0x07);
    uint8_t bit = (inst.opcode >> 4) & 0x07;

    uint8_t io_val = m_mcu.memory().read_io(addr);
    if (io_val & (1 << bit)) {
        // Skip next instruction
        m_state.PC += 1;
    }
    return true;
}

bool ATmegaInterpreter::exec_LPM(const Instruction& inst) {
    // Load from program memory using Z pointer
    (void)inst;

    // Z pointer = r31:r30
    uint16_t z_addr = (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t data = m_flash[z_addr >> 1] & 0xFF;  // Low byte

    // Store in r0 (default)
    m_mcu.memory().gp_registers[0] = data;

    return true;
}

bool ATmegaInterpreter::exec_NOP_other(const Instruction& inst) {
    (void)inst;
    return true;
}

// ========================================
// MULTIPLICATION INSTRUCTIONS
// ========================================

bool ATmegaInterpreter::exec_MUL(const Instruction& inst) {
    // MUL: Rd * Rr -> R1:R0
    uint8_t rd = get_rd(inst.opcode);
    uint8_t rr = inst.opcode & 0x0F;

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];
    uint8_t rr_val = m_mcu.memory().gp_registers[rr];

    uint16_t result = rd_val * rr_val;

    m_mcu.memory().gp_registers[0] = result & 0xFF;     // R0 = low byte
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;  // R1 = high byte

    // Set flags
    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;  // C = MSB of result

    return true;
}

bool ATmegaInterpreter::exec_MULS(const Instruction& inst) {
    // MULS: Signed Rd * Signed Rr -> R1:R0
    uint8_t rd = 16 + ((inst.opcode >> 4) & 0x7);
    uint8_t rr = inst.opcode & 0x7;

    int8_t rd_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rd]);
    int8_t rr_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rr]);

    int16_t result = rd_val * rr_val;

    m_mcu.memory().gp_registers[0] = result & 0xFF;
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;

    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;

    return true;
}

bool ATmegaInterpreter::exec_MULSU(const Instruction& inst) {
    // MULSU: Signed Rd * Unsigned Rr -> R1:R0
    uint8_t rd = 16 + ((inst.opcode >> 4) & 0x7);
    uint8_t rr = 16 + (inst.opcode & 0x7);

    int8_t rd_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rd]);
    uint8_t rr_val = m_mcu.memory().gp_registers[rr];

    int16_t result = rd_val * rr_val;

    m_mcu.memory().gp_registers[0] = result & 0xFF;
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;

    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;

    return true;
}

bool ATmegaInterpreter::exec_FMUL(const Instruction& inst) {
    // FMUL: Unsigned Rd * Unsigned Rr -> R1:R0 (shifted left by 1)
    uint8_t rd = 16 + ((inst.opcode >> 4) & 0x7);
    uint8_t rr = 16 + (inst.opcode & 0x7);

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];
    uint8_t rr_val = m_mcu.memory().gp_registers[rr];

    uint16_t result = (rd_val * rr_val * 2) & 0xFFFF;

    m_mcu.memory().gp_registers[0] = result & 0xFF;
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;

    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;

    return true;
}

bool ATmegaInterpreter::exec_FMULS(const Instruction& inst) {
    // FMULS: Signed Rd * Signed Rr -> R1:R0 (shifted left by 1)
    uint8_t rd = 16 + ((inst.opcode >> 4) & 0x7);
    uint8_t rr = 16 + (inst.opcode & 0x7);

    int8_t rd_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rd]);
    int8_t rr_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rr]);

    int16_t result = (rd_val * rr_val * 2) & 0xFFFF;

    m_mcu.memory().gp_registers[0] = result & 0xFF;
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;

    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;

    return true;
}

bool ATmegaInterpreter::exec_FMULSU(const Instruction& inst) {
    // FMULSU: Signed Rd * Unsigned Rr -> R1:R0 (shifted left by 1)
    uint8_t rd = 16 + ((inst.opcode >> 4) & 0x7);
    uint8_t rr = 16 + (inst.opcode & 0x7);

    int8_t rd_val = static_cast<int8_t>(m_mcu.memory().gp_registers[rd]);
    uint8_t rr_val = m_mcu.memory().gp_registers[rr];

    int16_t result = (rd_val * rr_val * 2) & 0xFFFF;

    m_mcu.memory().gp_registers[0] = result & 0xFF;
    m_mcu.memory().gp_registers[1] = (result >> 8) & 0xFF;

    m_state.SREG.Z = (result == 0);
    m_state.SREG.C = (result & 0x8000) != 0;

    return true;
}

// ========================================
// CONTROL FLOW INSTRUCTIONS
// ========================================

bool ATmegaInterpreter::exec_SLEEP(const Instruction& inst) {
    (void)inst;
    // Enter sleep mode - in simulation, just continue
    // In real hardware, this would wait for interrupt
    return true;
}

bool ATmegaInterpreter::exec_WDR(const Instruction& inst) {
    (void)inst;
    // Watchdog Reset - in simulation, just continue
    return true;
}

bool ATmegaInterpreter::exec_ELPM(const Instruction& inst) {
    // Extended Load from Program Memory using RAMPZ:Z
    (void)inst;

    // RAMPZ:Z pointer for extended addressing (>64K)
    uint8_t rampz = m_mcu.memory().gp_registers[33];  // RAMPZ is R33
    uint16_t z_addr = (rampz << 16) | (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t data = m_flash[(z_addr >> 1) % m_flash.size()] & 0xFF;

    // Store in r0 (default)
    m_mcu.memory().gp_registers[0] = data;

    return true;
}

bool ATmegaInterpreter::exec_LSR(const Instruction& inst) {
    // LSR: Logical Shift Right
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];

    m_state.SREG.C = (rd_val & 0x01) != 0;  // C = LSB

    uint8_t result = rd_val >> 1;
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);  // N = 0 always for LSR
    m_state.SREG.V = m_state.SREG.N ^ m_state.SREG.C;  // V = N ^ C
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_ROR(const Instruction& inst) {
    // ROR: Rotate Right through Carry
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];
    bool old_c = m_state.SREG.C;

    m_state.SREG.C = (rd_val & 0x01) != 0;  // C = LSB

    uint8_t result = (rd_val >> 1) | (old_c ? 0x80 : 0x00);
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    m_state.SREG.V = m_state.SREG.N ^ m_state.SREG.C;
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_ASR(const Instruction& inst) {
    // ASR: Arithmetic Shift Right
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];

    m_state.SREG.C = (rd_val & 0x01) != 0;  // C = LSB

    // Preserve sign bit (bit 7)
    uint8_t result = (rd_val >> 1) | (rd_val & 0x80);
    m_mcu.memory().gp_registers[rd] = result;

    update_z_flag(result);
    update_n_flag(result);
    m_state.SREG.V = m_state.SREG.N ^ m_state.SREG.C;
    update_s_flag();

    return true;
}

bool ATmegaInterpreter::exec_SWAP(const Instruction& inst) {
    // SWAP: Swap nibbles in register
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint8_t rd_val = m_mcu.memory().gp_registers[rd];
    uint8_t result = ((rd_val & 0x0F) << 4) | ((rd_val & 0xF0) >> 4);
    m_mcu.memory().gp_registers[rd] = result;

    return true;
}

bool ATmegaInterpreter::exec_SBI(const Instruction& inst) {
    // SBI: Set Bit in I/O Register
    uint8_t addr = ((inst.opcode >> 3) & 0x18) | (inst.opcode & 0x07);  // I/O address
    uint8_t bit = (inst.opcode >> 4) & 0x07;

    uint8_t io_val = m_mcu.memory().read_io(addr);
    io_val |= (1 << bit);
    m_mcu.memory().write_io(addr, io_val);

    return true;
}

bool ATmegaInterpreter::exec_CBI(const Instruction& inst) {
    // CBI: Clear Bit in I/O Register
    uint8_t addr = ((inst.opcode >> 3) & 0x18) | (inst.opcode & 0x07);
    uint8_t bit = (inst.opcode >> 4) & 0x07;

    uint8_t io_val = m_mcu.memory().read_io(addr);
    io_val &= ~(1 << bit);
    m_mcu.memory().write_io(addr, io_val);

    return true;
}

// ========================================
// BRANCH INSTRUCTIONS (Additional)
// ========================================

bool ATmegaInterpreter::exec_BRCS(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.C) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRCC(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.C) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRGE(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.S) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRLT(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.S) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRHS(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.H) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRHC(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.H) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRTS(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.T) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRTC(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.T) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRVS(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.V) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRVC(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.V) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRIE(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (m_state.SREG.I) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRID(const Instruction& inst) {
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    if (!m_state.SREG.I) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRBC(const Instruction& inst) {
    // BRBC: Branch if Bit in SREG is Clear
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    uint8_t bit = (inst.opcode >> 3) & 0x07;

    bool bit_set = false;
    switch (bit) {
        case 0: bit_set = m_state.SREG.C; break;
        case 1: bit_set = m_state.SREG.Z; break;
        case 2: bit_set = m_state.SREG.N; break;
        case 3: bit_set = m_state.SREG.V; break;
        case 4: bit_set = m_state.SREG.S; break;
        case 5: bit_set = m_state.SREG.H; break;
        case 6: bit_set = m_state.SREG.T; break;
        case 7: bit_set = m_state.SREG.I; break;
    }

    if (!bit_set) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

bool ATmegaInterpreter::exec_BRBS(const Instruction& inst) {
    // BRBS: Branch if Bit in SREG is Set
    int8_t offset = static_cast<int8_t>(inst.opcode & 0xFF);
    uint8_t bit = (inst.opcode >> 3) & 0x07;

    bool bit_set = false;
    switch (bit) {
        case 0: bit_set = m_state.SREG.C; break;
        case 1: bit_set = m_state.SREG.Z; break;
        case 2: bit_set = m_state.SREG.N; break;
        case 3: bit_set = m_state.SREG.V; break;
        case 4: bit_set = m_state.SREG.S; break;
        case 5: bit_set = m_state.SREG.H; break;
        case 6: bit_set = m_state.SREG.T; break;
        case 7: bit_set = m_state.SREG.I; break;
    }

    if (bit_set) {
        m_state.PC = m_state.PC + offset;
    }
    return true;
}

// ========================================
// BIT INSTRUCTIONS
// ========================================

bool ATmegaInterpreter::exec_SEC(const Instruction& inst) {
    (void)inst;
    m_state.SREG.C = true;
    return true;
}

bool ATmegaInterpreter::exec_SEZ(const Instruction& inst) {
    (void)inst;
    m_state.SREG.Z = true;
    return true;
}

bool ATmegaInterpreter::exec_SEN(const Instruction& inst) {
    (void)inst;
    m_state.SREG.N = true;
    return true;
}

bool ATmegaInterpreter::exec_SEV(const Instruction& inst) {
    (void)inst;
    m_state.SREG.V = true;
    return true;
}

bool ATmegaInterpreter::exec_SES(const Instruction& inst) {
    (void)inst;
    m_state.SREG.S = true;
    return true;
}

bool ATmegaInterpreter::exec_SEH(const Instruction& inst) {
    (void)inst;
    m_state.SREG.H = true;
    return true;
}

bool ATmegaInterpreter::exec_SET(const Instruction& inst) {
    (void)inst;
    m_state.SREG.T = true;
    return true;
}

bool ATmegaInterpreter::exec_SEI(const Instruction& inst) {
    (void)inst;
    m_state.SREG.I = true;
    return true;
}

bool ATmegaInterpreter::exec_CLC(const Instruction& inst) {
    (void)inst;
    m_state.SREG.C = false;
    return true;
}

bool ATmegaInterpreter::exec_CLZ(const Instruction& inst) {
    (void)inst;
    m_state.SREG.Z = false;
    return true;
}

bool ATmegaInterpreter::exec_CLN(const Instruction& inst) {
    (void)inst;
    m_state.SREG.N = false;
    return true;
}

bool ATmegaInterpreter::exec_CLV(const Instruction& inst) {
    (void)inst;
    m_state.SREG.V = false;
    return true;
}

bool ATmegaInterpreter::exec_CLS(const Instruction& inst) {
    (void)inst;
    m_state.SREG.S = false;
    return true;
}

bool ATmegaInterpreter::exec_CLH(const Instruction& inst) {
    (void)inst;
    m_state.SREG.H = false;
    return true;
}

bool ATmegaInterpreter::exec_CLT(const Instruction& inst) {
    (void)inst;
    m_state.SREG.T = false;
    return true;
}

bool ATmegaInterpreter::exec_CLI(const Instruction& inst) {
    (void)inst;
    m_state.SREG.I = false;
    return true;
}

bool ATmegaInterpreter::exec_BST(const Instruction& inst) {
    // BST: Bit Store from Register to T flag
    uint8_t rd = (inst.opcode >> 4) & 0x1F;
    uint8_t bit = inst.opcode & 0x07;

    m_state.SREG.T = (m_mcu.memory().gp_registers[rd] & (1 << bit)) != 0;
    return true;
}

bool ATmegaInterpreter::exec_BLD(const Instruction& inst) {
    // BLD: Bit Load from T flag to Register
    uint8_t rd = (inst.opcode >> 4) & 0x1F;
    uint8_t bit = inst.opcode & 0x07;

    if (m_state.SREG.T) {
        m_mcu.memory().gp_registers[rd] |= (1 << bit);
    } else {
        m_mcu.memory().gp_registers[rd] &= ~(1 << bit);
    }
    return true;
}

// ========================================
// LOAD/STORE INSTRUCTIONS
// ========================================

bool ATmegaInterpreter::exec_MOVW(const Instruction& inst) {
    // MOVW: Copy Register Word (R+1:R <- Rr+1:Rr)
    uint8_t rd = ((inst.opcode >> 4) & 0x0F) * 2;  // Even register
    uint8_t rr = (inst.opcode & 0x0F) * 2;         // Even register

    m_mcu.memory().gp_registers[rd] = m_mcu.memory().gp_registers[rr];
    m_mcu.memory().gp_registers[rd + 1] = m_mcu.memory().gp_registers[rr + 1];

    return true;
}

bool ATmegaInterpreter::exec_ADIW(const Instruction& inst) {
    // ADIW: Add Immediate to Word (Rd+1:Rd <- Rd+1:Rd + K)
    // Valid for Rd = 24, 26, 28, 30 (X, Y, Z pairs)

    uint8_t q = (inst.opcode >> 3) & 0x06;  // q = 0, 2, 4, 6
    uint8_t rd = 24 + q;  // R24:R25 for X, R26:R27 for Y, R28:R29 for Z, R30:R31

    uint8_t k = ((inst.opcode >> 2) & 0x30) | (inst.opcode & 0x0F);

    uint16_t word_val = m_mcu.memory().gp_registers[rd] |
                       (m_mcu.memory().gp_registers[rd + 1] << 8);
    uint16_t result = word_val + k;

    m_mcu.memory().gp_registers[rd] = result & 0xFF;
    m_mcu.memory().gp_registers[rd + 1] = (result >> 8) & 0xFF;

    // Update flags
    m_state.SREG.V = ((~word_val & result) & 0x8000) != 0;  // Overflow
    m_state.SREG.Z = (result == 0);
    m_state.SREG.N = (result & 0x8000) != 0;
    m_state.SREG.C = ((~word_val & k & 0x8000) != 0) || ((word_val & k & 0x8000) != 0) ||
                     ((k & ~result & 0x8000) != 0);
    m_state.SREG.S = m_state.SREG.N != m_state.SREG.V;

    return true;
}

bool ATmegaInterpreter::exec_SBIW(const Instruction& inst) {
    // SBIW: Subtract Immediate from Word
    uint8_t q = (inst.opcode >> 3) & 0x06;
    uint8_t rd = 24 + q;

    uint8_t k = ((inst.opcode >> 2) & 0x30) | (inst.opcode & 0x0F);

    uint16_t word_val = m_mcu.memory().gp_registers[rd] |
                       (m_mcu.memory().gp_registers[rd + 1] << 8);
    uint16_t result = word_val - k;

    m_mcu.memory().gp_registers[rd] = result & 0xFF;
    m_mcu.memory().gp_registers[rd + 1] = (result >> 8) & 0xFF;

    // Update flags
    m_state.SREG.V = ((word_val & ~result) & 0x8000) != 0;
    m_state.SREG.Z = (result == 0);
    m_state.SREG.N = (result & 0x8000) != 0;
    m_state.SREG.C = ((~word_val & k & 0x8000) != 0) || ((word_val & ~result & 0x8000) != 0) ||
                     ((k & ~result & 0x8000) != 0);
    m_state.SREG.S = m_state.SREG.N != m_state.SREG.V;

    return true;
}

bool ATmegaInterpreter::exec_XCH(const Instruction& inst) {
    // XCH: Exchange Z with Rd (not in all AVRs, but in ATmega328P)
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint16_t z_addr = (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t mem_val = m_flash[z_addr >> 1] & 0xFF;
    uint8_t reg_val = m_mcu.memory().gp_registers[rd];

    // Exchange
    m_mcu.memory().gp_registers[rd] = mem_val;
    // Note: Can't write to flash in simulation

    return true;
}

bool ATmegaInterpreter::exec_LAS(const Instruction& inst) {
    // LAS: Load and Set from Z
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint16_t z_addr = (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t mem_val = m_flash[z_addr >> 1] & 0xFF;
    m_mcu.memory().gp_registers[rd] |= mem_val;

    return true;
}

bool ATmegaInterpreter::exec_LAC(const Instruction& inst) {
    // LAC: Load and Clear from Z
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint16_t z_addr = (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t mem_val = m_flash[z_addr >> 1] & 0xFF;
    m_mcu.memory().gp_registers[rd] &= ~mem_val;

    return true;
}

bool ATmegaInterpreter::exec_LAT(const Instruction& inst) {
    // LAT: Load and Toggle from Z
    uint8_t rd = (inst.opcode >> 4) & 0x1F;

    uint16_t z_addr = (m_mcu.memory().gp_registers[31] << 8) |
                      m_mcu.memory().gp_registers[30];

    uint8_t mem_val = m_flash[z_addr >> 1] & 0xFF;
    m_mcu.memory().gp_registers[rd] ^= mem_val;

    return true;
}

// ===== Status Flag Helpers =====

void ATmegaInterpreter::update_z_flag(uint8_t result) {
    m_state.SREG.Z = (result == 0);
}

void ATmegaInterpreter::update_n_flag(uint8_t result) {
    m_state.SREG.N = (result & 0x80) != 0;
}

void ATmegaInterpreter::update_v_flag(uint8_t a, uint8_t b, uint8_t result, bool is_subtraction) {
    if (is_subtraction) {
        // Overflow for subtraction: (a < 0 && b >= 0 && result > 0) || (a >= 0 && b < 0 && result < 0)
        bool a_neg = (a & 0x80) != 0;
        bool b_neg = (b & 0x80) != 0;
        bool r_neg = (result & 0x80) != 0;
        m_state.SREG.V = (a_neg && !b_neg && !r_neg) || (!a_neg && b_neg && r_neg);
    } else {
        // Overflow for addition
        bool a_neg = (a & 0x80) != 0;
        bool b_neg = (b & 0x80) != 0;
        bool r_neg = (result & 0x80) != 0;
        m_state.SREG.V = (!a_neg && !b_neg && r_neg) || (a_neg && b_neg && !r_neg);
    }
}

void ATmegaInterpreter::update_c_flag(uint8_t a, uint8_t b, bool is_subtraction) {
    if (is_subtraction) {
        m_state.SREG.C = (a < b);
    } else {
        uint16_t sum = a + b;
        m_state.SREG.C = (sum > 0xFF);
    }
}

void ATmegaInterpreter::update_s_flag() {
    m_state.SREG.S = m_state.SREG.N != m_state.SREG.V;
}

void ATmegaInterpreter::update_h_flag(uint8_t a, uint8_t b, bool is_subtraction) {
    if (is_subtraction) {
        // Half carry for subtraction: bit 3 borrow
        m_state.SREG.H = ((a & 0x08) < (b & 0x08));
    } else {
        // Half carry for addition: bit 3 carry
        m_state.SREG.H = ((a & 0x08) + (b & 0x08)) > 0x08;
    }
}

// ===== Operand Extraction Helpers =====

uint8_t ATmegaInterpreter::get_rd(uint16_t opcode) const {
    return (opcode >> 4) & 0x1F;
}

uint8_t ATmegaInterpreter::get_rr(uint16_t opcode) const {
    return opcode & 0x0F;
}

uint8_t ATmegaInterpreter::get_k(uint16_t opcode) const {
    uint8_t k = ((opcode >> 4) & 0xF0) | (opcode & 0x0F);
    return k;
}

uint8_t ATmegaInterpreter::get_io_addr(uint16_t opcode) const {
    return ((opcode >> 5) & 0x30) | (opcode & 0x0F);
}

// ===== Breakpoint Management =====

void ATmegaInterpreter::set_breakpoint(uint16_t addr) {
    m_breakpoints[addr] = true;
    spdlog::debug("Breakpoint set at 0x{:04X}", addr);
}

void ATmegaInterpreter::clear_breakpoint(uint16_t addr) {
    m_breakpoints.erase(addr);
}

void ATmegaInterpreter::clear_all_breakpoints() {
    m_breakpoints.clear();
}

bool ATmegaInterpreter::has_breakpoint(uint16_t addr) const {
    return m_breakpoints.find(addr) != m_breakpoints.end();
}

std::string ATmegaInterpreter::disassemble_current() const {
    if (m_state.PC >= m_flash.size()) {
        return "END_OF_MEMORY";
    }

    uint16_t opcode = m_flash[m_state.PC];

    // Simple disassembly
    std::ostringstream ss;
    ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << m_state.PC;
    ss << ": 0x" << std::setw(4) << opcode;

    // Find mnemonic
    ExecFunc handler = m_instruction_table[opcode];

    // Check if it's a known instruction or NOP_other
    auto jmp_handler = &ATmegaInterpreter::exec_JMP;
    auto ijmp_handler = &ATmegaInterpreter::exec_IJMP;
    auto nop_handler = &ATmegaInterpreter::exec_NOP_other;

    if (handler == nop_handler && opcode != 0x0000) {
        ss << " UNKNOWN";
    } else if (handler == jmp_handler) {
        // JMP - show target address
        uint16_t second_word = m_flash[m_state.PC + 1];
        uint32_t target = second_word;
        ss << " JMP 0x" << std::setw(4) << target;
    } else if (handler == ijmp_handler) {
        ss << " IJMP";
    } else {
        // Known instruction - try to identify it
        ss << " ";
        if (opcode == 0x0000) ss << "NOP";
        else if ((opcode & 0xF000) == 0xE000) ss << "LDI";
        else if ((opcode & 0xFC00) == 0x1C00) ss << "ADC";
        else if ((opcode & 0xFC00) == 0x0C00) ss << "ADD";
        else if ((opcode & 0xFC00) == 0x0400) ss << "CPC";
        else if ((opcode & 0xFE00) == 0x1400) ss << "CP";
        else if ((opcode & 0xF000) == 0x3000) ss << "CPI";
        else if ((opcode & 0xFC00) == 0x2000) ss << "AND";
        else if ((opcode & 0xFC00) == 0x2400) ss << "EOR";
        else if ((opcode & 0xFC00) == 0x2800) ss << "OR";
        else if ((opcode & 0xFE0F) == 0x940C || (opcode & 0xFE0F) == 0x940D) ss << "JMP";
        else if ((opcode & 0xF000) == 0xC000) ss << "RJMP";
        else if ((opcode & 0xF000) == 0xD000) ss << "RCALL";
        else if ((opcode & 0xFC00) == 0x9000 || (opcode & 0xFC00) == 0x9100) ss << "LDS";
        else if ((opcode & 0xFC00) == 0x9200 || (opcode & 0xFC00) == 0x9300) ss << "STS";
        else if ((opcode & 0xF800) == 0xB800) ss << "OUT";
        else if ((opcode & 0xF800) == 0xB000) ss << "IN";
        else if ((opcode & 0xFC00) == 0x1800) ss << "SUB";
        else if ((opcode & 0xFC00) == 0x0800) ss << "SBC";
        else if ((opcode & 0xF000) == 0x4000) ss << "SBCI";
        else if ((opcode & 0xF000) == 0x5000) ss << "SUBI";
        else if ((opcode & 0xF000) == 0x6000) ss << "ORI";
        else if ((opcode & 0xF000) == 0x7000) ss << "ANDI";
        else if ((opcode & 0xFE0E) == 0x940E) ss << "IJMP";
        else if ((opcode & 0xFE0E) == 0x940F) ss << "ICALL";
        else if ((opcode & 0xFC00) == 0x2C00) ss << "MOV";
        else if ((opcode & 0xF800) == 0x9800 || (opcode & 0xF800) == 0x9000) ss << "LD";
        else if ((opcode & 0xF800) == 0x9A00 || (opcode & 0xF800) == 0x9200) ss << "ST";
        else if ((opcode & 0xFF00) == 0x9500) ss << "PUSH/POP";
        else if ((opcode & 0xFF0F) == 0x9403) ss << "INC";
        else if ((opcode & 0xFF0F) == 0x940A) ss << "DEC";
        else if ((opcode & 0xFF0F) == 0x9400) ss << "COM";
        else if ((opcode & 0xFF0F) == 0x9401) ss << "NEG";
        else if ((opcode & 0xFF0F) == 0x9406) ss << "LSR";
        else if ((opcode & 0xFF0F) == 0x9407) ss << "ROR";
        else if ((opcode & 0xFF0F) == 0x9405) ss << "ASR";
        else if ((opcode & 0xFF0F) == 0x9402) ss << "SWAP";
        else if ((opcode & 0xFC00) == 0x9C00) ss << "MUL";
        else if ((opcode & 0xFF00) == 0x0200) ss << "MULS";
        else if ((opcode & 0xF000) == 0xF000) ss << "BREQ/BRNE/BRxx";
        else ss << "KNOWN";
    }

    return ss.str();
}

} // namespace mechatron
