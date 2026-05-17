#pragma once

#include "QEMUInterface.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <map>
#include <array>

namespace mechatron {

/**
 * @brief ATmega328P Instruction Interpreter
 *
 * Simple instruction interpreter for ATmega328P MCU.
 * Supports common instructions used in Arduino sketches.
 *
 * This allows real Arduino .hex files to run in simulation mode
 * without requiring QEMU to be installed.
 */
class ATmegaInterpreter {
public:
    struct InterpreterState {
        // Program counter (flash address)
        uint32_t PC = 0;

        // Stack pointer
        uint16_t SP = 0xFFFF;

        // Status register (SREG)
        struct {
            bool C = false;  // Carry
            bool Z = false;  // Zero
            bool N = false;  // Negative
            bool V = false;  // Overflow
            bool S = false;  // Sign
            bool H = false;  // Half carry
            bool T = false;  // Transfer
            bool I = false;  // Interrupt
        } SREG;

        // Cycle counter
        uint64_t cycles = 0;
    };

    ATmegaInterpreter(QEMUInterface& mcu);
    ~ATmegaInterpreter() = default;

    /**
     * Load firmware from Intel HEX file
     */
    bool load_firmware(const std::string& path);

    /**
     * Reset interpreter state
     */
    void reset();

    /**
     * Execute one instruction
     * @return true if execution should continue, false if halted
     */
    bool step();

    /**
     * Execute N instructions
     */
    bool step_instructions(uint32_t count);

    /**
     * Execute for a specified duration (microseconds)
     */
    void execute_for_us(uint32_t duration_us);

    /**
     * Get current state
     */
    const InterpreterState& state() const { return m_state; }

    /**
     * Get disassembly of current instruction
     */
    std::string disassemble_current() const;

    /**
     * Check if firmware is loaded
     */
    bool is_loaded() const { return !m_flash.empty(); }

    /**
     * Get current instruction count executed
     */
    uint64_t instruction_count() const { return m_instruction_count; }

    // Breakpoint management
    void set_breakpoint(uint32_t addr);
    void clear_breakpoint(uint32_t addr);
    void clear_all_breakpoints();
    bool has_breakpoint(uint32_t addr) const;

private:
    // Instruction decoding
    struct Instruction {
        uint16_t opcode;
        uint16_t operands;
        std::string mnemonic;
        uint8_t size;  // Instruction size in words (2 bytes)
        uint8_t cycles;  // Execution cycles
    };

    Instruction decode_instruction(uint32_t addr) const;
    bool execute_instruction(const Instruction& inst);

    // Instruction implementations
    bool exec_NOP(const Instruction& inst);
    bool exec_MOV(const Instruction& inst);
    bool exec_LDI(const Instruction& inst);
    bool exec_STS(const Instruction& inst);
    bool exec_LDS(const Instruction& inst);
    bool exec_ST(const Instruction& inst);
    bool exec_LD(const Instruction& inst);
    bool exec_PUSH(const Instruction& inst);
    bool exec_POP(const Instruction& inst);
    bool exec_IN(const Instruction& inst);
    bool exec_OUT(const Instruction& inst);
    bool exec_ANDI(const Instruction& inst);
    bool exec_ORI(const Instruction& inst);
    bool exec_EOR(const Instruction& inst);
    bool exec_AND(const Instruction& inst);
    bool exec_OR(const Instruction& inst);
    bool exec_COM(const Instruction& inst);
    bool exec_NEG(const Instruction& inst);
    bool exec_INC(const Instruction& inst);
    bool exec_DEC(const Instruction& inst);
    bool exec_ADD(const Instruction& inst);
    bool exec_ADC(const Instruction& inst);
    bool exec_SUB(const Instruction& inst);
    bool exec_SBC(const Instruction& inst);
    bool exec_SUBI(const Instruction& inst);
    bool exec_SBCI(const Instruction& inst);
    bool exec_RJMP(const Instruction& inst);
    bool exec_RCALL(const Instruction& inst);
    bool exec_RET(const Instruction& inst);
    bool exec_RETI(const Instruction& inst);
    bool exec_CALL(const Instruction& inst);
    bool exec_JMP(const Instruction& inst);
    bool exec_IJMP(const Instruction& inst);
    bool exec_ICALL(const Instruction& inst);
    bool exec_EIJMP(const Instruction& inst);
    bool exec_EICALL(const Instruction& inst);
    bool exec_CP(const Instruction& inst);
    bool exec_CPC(const Instruction& inst);
    bool exec_CPSE(const Instruction& inst);
    bool exec_CPI(const Instruction& inst);
    bool exec_BREQ(const Instruction& inst);
    bool exec_BRNE(const Instruction& inst);
    bool exec_BRSH(const Instruction& inst);
    bool exec_BRLO(const Instruction& inst);
    bool exec_BRMI(const Instruction& inst);
    bool exec_BRPL(const Instruction& inst);
    bool exec_SBRS(const Instruction& inst);
    bool exec_SBRC(const Instruction& inst);
    bool exec_SBIC(const Instruction& inst);
    bool exec_SBIS(const Instruction& inst);
    bool exec_LPM(const Instruction& inst);
    bool exec_NOP_other(const Instruction& inst);

    // Multiplication instructions
    bool exec_MUL(const Instruction& inst);
    bool exec_MULS(const Instruction& inst);
    bool exec_MULSU(const Instruction& inst);
    bool exec_FMUL(const Instruction& inst);
    bool exec_FMULS(const Instruction& inst);
    bool exec_FMULSU(const Instruction& inst);

    // Control flow instructions
    bool exec_SLEEP(const Instruction& inst);
    bool exec_WDR(const Instruction& inst);
    bool exec_ELPM(const Instruction& inst);

    // Shift and rotate instructions
    bool exec_LSR(const Instruction& inst);
    bool exec_ROR(const Instruction& inst);
    bool exec_ASR(const Instruction& inst);
    bool exec_SWAP(const Instruction& inst);

    // I/O bit instructions
    bool exec_SBI(const Instruction& inst);
    bool exec_CBI(const Instruction& inst);

    // Branch instructions (additional)
    bool exec_BRCS(const Instruction& inst);
    bool exec_BRCC(const Instruction& inst);
    bool exec_BRGE(const Instruction& inst);
    bool exec_BRLT(const Instruction& inst);
    bool exec_BRHS(const Instruction& inst);
    bool exec_BRHC(const Instruction& inst);
    bool exec_BRTS(const Instruction& inst);
    bool exec_BRTC(const Instruction& inst);
    bool exec_BRVS(const Instruction& inst);
    bool exec_BRVC(const Instruction& inst);
    bool exec_BRIE(const Instruction& inst);
    bool exec_BRID(const Instruction& inst);
    bool exec_BRBC(const Instruction& inst);
    bool exec_BRBS(const Instruction& inst);

    // Flag instructions
    bool exec_SEC(const Instruction& inst);
    bool exec_SEZ(const Instruction& inst);
    bool exec_SEN(const Instruction& inst);
    bool exec_SEV(const Instruction& inst);
    bool exec_SES(const Instruction& inst);
    bool exec_SEH(const Instruction& inst);
    bool exec_SET(const Instruction& inst);
    bool exec_SEI(const Instruction& inst);
    bool exec_CLC(const Instruction& inst);
    bool exec_CLZ(const Instruction& inst);
    bool exec_CLN(const Instruction& inst);
    bool exec_CLV(const Instruction& inst);
    bool exec_CLS(const Instruction& inst);
    bool exec_CLH(const Instruction& inst);
    bool exec_CLT(const Instruction& inst);
    bool exec_CLI(const Instruction& inst);
    bool exec_BST(const Instruction& inst);
    bool exec_BLD(const Instruction& inst);

    // Load/Store instructions
    bool exec_MOVW(const Instruction& inst);
    bool exec_ADIW(const Instruction& inst);
    bool exec_SBIW(const Instruction& inst);
    bool exec_XCH(const Instruction& inst);
    bool exec_LAS(const Instruction& inst);
    bool exec_LAC(const Instruction& inst);
    bool exec_LAT(const Instruction& inst);

    // Status register helpers
    void update_z_flag(uint8_t result);
    void update_n_flag(uint8_t result);
    void update_v_flag(uint8_t a, uint8_t b, uint8_t result, bool is_subtraction);
    void update_c_flag(uint8_t a, uint8_t b, bool is_subtraction);
    void update_s_flag();
    void update_h_flag(uint8_t a, uint8_t b, bool is_subtraction);

    // Arduino semantics: when firmware writes directly to a PWM-capable PORT bit (e.g. digitalWrite),
    // the core turns off PWM for that channel first. We emulate that behavior here to avoid "stuck PWM"
    // when the turnOffPWM path isn't modeled perfectly by the interpreter yet.
    void maybe_disable_pwm_on_port_write(uint16_t data_addr, uint8_t new_port_value);

    // Operand extraction helpers
    uint8_t get_rd(uint16_t opcode) const;
    uint8_t get_rr(uint16_t opcode) const;
    uint8_t get_k(uint16_t opcode) const;  // Immediate constant
    int8_t get_q(int16_t offset) const;   // Branch offset
    uint8_t get_io_addr(uint16_t opcode) const;

    // Reference to MCU interface
    QEMUInterface& m_mcu;

    // Flash memory (program memory) - 32KB for ATmega328P
    std::vector<uint16_t> m_flash;  // Words (16-bit)

    // Interpreter state
    InterpreterState m_state;
    uint64_t m_instruction_count = 0;

    // Breakpoints
    std::map<uint32_t, bool> m_breakpoints;
    uint32_t m_timer0_cycle_accum = 0;
    uint8_t m_cycle_adjust = 0;

    // Instruction table (for quick dispatch) - use array for O(1) lookup
    using ExecFunc = bool (ATmegaInterpreter::*)(const Instruction&);
    std::array<ExecFunc, 65536> m_instruction_table;

    void build_instruction_table();
    void update_timers(uint8_t cycles);
    void service_interrupt(uint8_t vector_index);
};

} // namespace mechatron
