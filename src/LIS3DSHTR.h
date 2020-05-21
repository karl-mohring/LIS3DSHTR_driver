#ifndef LIS3DSHTR_DRIVER_H
#define LIS3DSHTR_DRIVER_H

////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>

////////////////////////////////////////////////////////////////////////////////

const uint8_t DEFAULT_I2C_ADDRESS = 0x1E;

////////////////////////////////////////////////////////////////////////////////

enum LIS3DSHTR_COMMS_MODE { LIS3DSHTR_I2C_MODE, LIS3DSHTR_SPI_MODE };

typedef enum LIS3DSHTR_STATE_MACHINES {
    LIS3DSHTR_STATE_MACHINE_1 = 0,
    LIS3DSHTR_STATE_MACHINE_2 = 1,
} lis3dshtr_state_machine_selection_t;

typedef enum LIS3DSHTR_AXES {
    LIS3DSHTR_X_AXIS,
    LIS3DSHTR_Y_AXIS,
    LIS3DSHTR_Z_AXIS,
} lis3dshtr_axis_selection_t;

////////////////////////////////////////////////////////////////////////////////
// OUT_T
/**
 * Temperature output register.
 * Temperature = 25 + int8_t(register)
 * The value is expressed as two's complement.
 */

const uint8_t LIS3DSHTR_TEMPERATURE_OFFSET = 25;

typedef int8_t lis3dshtr_raw_temp_t;
typedef int8_t lis3dshtr_temp_t;

////////////////////////////////////////////////////////////////////////////////
// INFO1, INFO2
/**
 * Read-only information registers
 * INFO1 = 0x21
 * INFO2 = 0x00
 */

typedef uint8_t lis3dshtr_info_t;

////////////////////////////////////////////////////////////////////////////////
// WHO_AM_I
/**
 * Who am I ID.
 */

typedef uint8_t lis3dshtr_who_am_i_t;
const uint8_t LIS3DSHTR_WHO_AM_I_ID = 0b111111;

////////////////////////////////////////////////////////////////////////////////
// OFF_X, OFF_Y, OFF_Z
/**
 * Offset compensation register for single axis.
 * Default value is 00h.
 * The value is expressed in two’s complement.
 *
 * Final acceleration output value is composed as:
 * Output(axis) = Measurement(axis) - OFFSET_x(axis) * 32
 * Where:
 * x = X, Y, Z-axis
 * Measurement(axis) = 16-bit raw data for X, Y, Z
 * OFFSET_x(axis) = Compensation value from OFF_X, OFF_Y, OFF_Z registers
 * OUTPUT(axis) = Acceleration value with offset compensation for output registers and state machine.
 *
 * According to the previous formula, the offset on each axis can be compensated from -4095 to 4096 LSB, with steps of 32 LSB
 */

typedef int8_t lis3dshtr_offset_t;

////////////////////////////////////////////////////////////////////////////////
// CS_X, CS_Y, CS_Z
/**
 * Constant shift for axis registers, signed value.
 * This value acts as a temporary offset in DIFF-mode for State Machine 2 only.
 */

typedef int8_t lis3dshtr_constant_shift_t;

////////////////////////////////////////////////////////////////////////////////
// LC
/**
 * 16-bit long-counter register for interrupt state machine program timing.
 * Little-endian.
 *
 * -01h = counting stopped
 *  00h = counter full: interrupt available and counter is set to -01.
 * >00h: counting
 *
 * This value is decreased whenever the DEC opcode is executed in the state machine and
 * the counter value is higher or equal to zero.
 * To stop counting, the value -01h must be written in these registers.
 * When the long counter is full (00h), the LONG bit is set to 1 in the STAT register (18h).
 * The following state for the long counter is -01h (counter stopped).
 * Reading the LC registers resets the LONG bit in the STAT register (18h) to the default value (0).
 */

typedef union {
    uint8_t bytes[2];
    uint16_t raw;
} lis3dshtr_long_counter_t;

// TIM1_n, TIM2_n, TIM3_n, TIM4_n, THRS1_n, THRS2_n, THRS3, TCn

typedef enum LIS3DSHTR_SHORT_COUNTERS { LIS3DSHTR_TIM4, LIS3DSHTR_TIM3 } lis3dshtr_short_counter_selection_t;

typedef enum LIS3DSHTR_LONG_COUNTERS { LIS3DSHTR_LC, LIS3DSHTR_TIM2, LIS3DSHTR_TIM1, LIS3DSHTR_TC } lis3dshtr_long_counter_selection_t;

typedef enum LIS3DSHTR_THRESHOLDS {
    LIS3DSHTR_THRS1,
    LIS3DSHTR_THRS2,
    LIS3DSHTR_THRS3,
} lis3dshtr_threshold_selection_t;

typedef uint8_t lis3dshtr_short_counter_t;  // 1 LSB = 1/ODR
typedef uint16_t lis3dshtr_long_counter_t;  // 1 LSB = 1/ODR
typedef uint16_t lis3dshtr_lc_t;
typedef int8_t lis3dshtr_threshold_t;  // 1 LSB = FS/128
typedef uint8_t lis3dshtr_thrs3_t;     // 1 LSB = FS/128

/**
 * THRS3 is the common threshold for overrun detection.
 * The value is always unsigned (ABS) regardless of the ABS settings in the SETT1/SETT2 registers.
 * So, the THRS3 value is symmetric to the zero level.
 * When the acceleration of any axis exceeds the THRS3 limit, the state machines are reset (PPx = RPx).
 * The reset of the state machines is enabled through the THR3_xA bits in the SETT1/SETT2 registers.
 */

////////////////////////////////////////////////////////////////////////////////
// STAT

typedef union {
    uint8_t raw;
    struct {
        bool is_data_ready : 1;                     // New data is ready in the output registers
        bool data_overrun_triggered : 1;            // New data has overwritten the previous set. Flag cleared when data is read
        bool sm2_interrupt_triggered : 1;           // Interrupt flag for SM1. Cleared on OUTS2 read
        bool sm1_interrupt_triggered : 1;           // Interrupt flag for SM2. Cleared on OUTS1 read
        bool sm2_is_awaiting_restart_from_sm1 : 1;  // State machine 2 has stopped and is waiting for restart request from SM1
        bool sm1_is_awaiting_restart_from_sm2 : 1;  // State machine 1 has stopped and is waiting for restart request from SM2
        bool is_waiting_after_outw_command : 1;     // Waiting on host action flag. Bit cleared whenever OUTS1 or OUTS2 is read
        bool long_counter_interrupt_triggered : 1;  // Reset to default by reading LC registers
    };

} lis3dshtr_stat_t;

////////////////////////////////////////////////////////////////////////////////
// PEAK1, PEAK2
/**
 * Peak 1 value for State Machine 1, default value: 00h.
 * Peak 2 value for State Machine 2, default value: 00h.
 * The peak register stores the highest absolute peak value detected.
 * The peak value is reset when the REL command occurs or a new initial start occurs.
 * The value of the peak counter is expressed in two’s complement.
 */

typedef int8_t lis3dshtr_peak_t;

////////////////////////////////////////////////////////////////////////////////
// VFC - Vector filter coefficients
/**
 * The vector filter is a 7th-order anti-symmetric FIR filter.
 * The transfer function of this filter is the following:
 * Xv_filt = (x0- x7) coeff0 + (x1-x6) coeff1+ (x2-x5) coeff2 + (x3-x4) coeff3
 * where:
 * coeff0 = VFC_4 (1Eh) register value;
 * coeff1 = VFC_3 (1Dh) register value;
 * coeff2 = VFC_2 (1Ch) register value;
 * coeff3 = VFC_1 (1Bh) register value.
 */

typedef enum LIS3DSHTR_VECTOR_COEFFICIENTS {
    LIS3DSHTR_VFC_1 = 0,
    LIS3DSHTR_VFC_2 = 1,
    LIS3DSHTR_VFC_3 = 2,
    LIS3DSHTR_VFC_4 = 3,

    LIS3DSHTR_VECTOR_COEFF_0 = LIS3DSHTR_VFC_4,
    LIS3DSHTR_VECTOR_COEFF_1 = LIS3DSHTR_VFC_3,
    LIS3DSHTR_VECTOR_COEFF_2 = LIS3DSHTR_VFC_2,
    LIS3DSHTR_VECTOR_COEFF_3 = LIS3DSHTR_VFC_1,
} lis3dshtr_vector_filter_coefficient_selection_t;

typedef int8_t lis3dshtr_vector_filter_coeff_t;

////////////////////////////////////////////////////////////////////////////////
// CTRL_REG4

typedef enum LIS3DSHTR_ODR_MODES {
    LIS3DSHTR_POWER_DOWN = 0,
    LIS3DSHTR_ODR_3_HZ = 1,
    LIS3DSHTR_ODR_6_HZ = 2,
    LIS3DSHTR_ODR_12_HZ = 3,
    LIS3DSHTR_ODR_25_HZ = 4,
    LIS3DSHTR_ODR_50_HZ = 5,
    LIS3DSHTR_ODR_100_HZ = 6,
    LIS3DSHTR_ODR_400_HZ = 7,
    LIS3DSHTR_ODR_800_HZ = 8,
    LIS3DSHTR_ODR_1600_HZ = 9,
} lis3dshtr_odr_t;

typedef union {
    uint8_t raw;
    struct {
        bool x_axis_enabled : 1;  // Data
        bool y_axis_enabled : 1;
        bool z_axis_enabled : 1;
        bool block_data_update_enabled : 1;
        lis3dshtr_odr_t output_data_rate : 4;
    }
} lis3dshtr_power_config_t;

////////////////////////////////////////////////////////////////////////////////
// CTRL_REG1, CTRL_REG2

typedef enum LIS3DSHTR_STATE_MACHINE_INTERRUPT_OUTPUT { LIS3DSHTR_SM_INTERRUPT_OUTPUT_INT1 = 0, LIS3DSHTR_SM_INTERRUPT_OUTPUT_INT2 = 1 } lis3dshtr_sm_interrupt_output_t;

typedef union {
    uint8_t raw;
    struct {
        bool state_machine_enabled : 1;  // Enable the state machine. Temporary memories and registers are left intact when disabled
        uint8_t _reserved0 : 2;
        lis3dshtr_sm_interrupt_output_t interrupt_output_channel : 1;  // Interrupt routing channel
        uint8_t _reserved1 : 1;
        uint8_t threshold_hysteresis : 3;  // Hysteresis to add or subtract from thresholds THRS1 and THSR2
    };

} lis3dshtr_state_machine_config_t;

////////////////////////////////////////////////////////////////////////////////
// CTRL_REG3

typedef union {
    uint8_t raw;
    struct {
        bool soft_reset_enabled : 1;             // Reset internal logic circuitry when enabled.
        bool vector_filter_enabled : 1;          // Enable the vector filter
        bool int1_signal_enabled : 1;            // Enabled the INT1 interrupt channel
        bool int2_signal_enabled : 1;            // Enable the INT2 interrupt channel
        bool pulsed_interrupts_enabled : 1;      // Use a pulsed interrupt. Latched interrupts used when disabled
        bool interrupt_active_high_enabled : 1;  // Trigger a logical HIGH when interrupts are triggered
        bool data_ready_interrupt_enabled : 1;   // Interrupt on INT1 when data is ready
    };
} lis3dshtr_interrupt_config_t;

////////////////////////////////////////////////////////////////////////////////
// CTRL_REG5

typedef enum LIS3DSHTR_AA_FILTER_BANDWIDTH {
    LIS3DSHTR_AA_FILTER_800_HZ = 0,
    LIS3DSHTR_AA_FILTER_200_HZ = 1,
    LIS3DSHTR_AA_FILTER_400_HZ = 2,
    LIS3DSHTR_AA_FILTER_50_HZ = 3,

} lis3dshtr_aa_filter_bandwidth_t;

typedef enum LIS3DSHTR_SELF_TEST_MODES {
    LIS3DSHTR_SELF_TEST_DISABLED = 0,
    LIS3DSHTR_SELF_TEST_POSITIVE_SIGN = 1,
    LIS3DSHTR_SELF_TEST_NEGATIVE_SIGN = 2,
} lis3dshtr_self_test_mode_t;

typedef enum LIS3DSHTR_FULL_SCALE_MODES {
    LIS3DSHTR_FULL_SCALE_2G = 0,
    LIS3DSHTR_FULL_SCALE_4G = 1,
    LIS3DSHTR_FULL_SCALE_6G = 2,
    LIS3DSHTR_FULL_SCALE_8G = 3,
    LIS3DSHTR_FULL_SCALE_16G = 4
} lis3dshtr_full_scale_mode_t;

typedef union {
    uint8_t raw;
    struct {
        bool spi_3_wire_enabled : 1;                              // Enable 3-wire SPI mode. 4-wire mode used when disabled.
        lis3dshtr_self_test_mode_t self_test_mode : 2;            // Self test mode/enable;
        lis3dshtr_full_scale_mode_t full_scale_mode : 3;          // Full-scale selection for measurements
        lis3dshtr_aa_filter_bandwidth_t aa_filter_bandwidth : 2;  // Anti-aliasing filter bandwidth
    };
} lis3dshtr_scale_config_t;

////////////////////////////////////////////////////////////////////////////////
// CTRL_REG6

typedef union {
    uint8_t raw;
    struct {
        bool boot_interrupt_on_int2_enabled : 1;                   // Trigger interrupt on INT2 on boot
        bool fifo_overrun_interrupt_on_int1_enabled : 1;           // Trigger interrupt on INT1 when FIFO buffer overruns
        bool fifo_watermark_interrupt_on_int1_enabled : 1;         // Trigger interrupt on INT1 when FIFO buffer reaches watermark level
        bool fifo_empty_interrupt_on_int1_enabled : 1;             // Trigger interrupt on INT1 when FIFO buffer is empty
        bool automatic_register_address_incrementing_enabled : 1;  // Automatically increase the register address during multiple byte read/writes
        bool stop_on_watermark_enabled : 1;                        // Stop filling FIFO once watermark is reached
        bool fifo_enabled : 1;                                     // Enable use of the FIFO buffer
        bool force_reboot_enabled : 1;                             // Force the device to reboot upon register write
    };
} lis3dshtr_fifo_config_t;  // CTRL_REG6

////////////////////////////////////////////////////////////////////////////////
// STATUS

typedef union {
    uint8_t raw;
    struct {
        bool x_data_is_available : 1;         // New data is available for the X-axis
        bool y_data_is_available : 1;         // New data is available for the Y-axis
        bool z_data_is_available : 1;         // New data is available for the Z-axis
        bool xyz_data_is_available : 1;       // New data is available for all X, Y, Z axes
        bool x_data_overrun_triggered : 1;    // New data has overwritten unread data on the X-axis
        bool y_data_overrun_triggered : 1;    // New data has overwritten unread data on the Y-axis
        bool z_data_overrun_triggered : 1;    // New data has overwritten unread data on the Z-axis
        bool xyz_data_overrun_triggered : 1;  // New data has overwritten unread data all X, Y, Z axes
    };
} lis3dshtr_status_t;

////////////////////////////////////////////////////////////////////////////////
// OUT_X, OUT_Y, OUT_Z

const uint8_t LIS3DSHTR_FIFO_SIZE = 32;
const uint8_t LIS3DSHTR_ENTRY_SIZE = 6;

typedef struct {
    int16_t x;  // Acceleration in X axis
    int16_t y;  // Acceleration in Y axis
    int16_t z;  // Acceleration in Z axis
} lis3dshtr_raw_data_t;

typedef struct {
    float roll;   // Roll in radians wrt. gravity
    float pitch;  // Pitch in radians wrt. gravity
} lis3dshtr_roll_pitch_data_t;

typedef lis3dshtr_raw_data_t data[LIS3DSHTR_FIFO_SIZE] lis3dshtr_fifo_data_t;

////////////////////////////////////////////////////////////////////////////////
// FIFO_CTRL

typedef enum LIS3DSHTR_FIFO_MODES {
    LIS3DSHTR_FIFO_BYPASS = 0b000,            // FIFO turned off
    LIS3DSHTR_FIFO_FIFO = 0b001,              // FIFO mode - Stop collecting when full
    LIS3DSHTR_FIFO_STREAM = 0b010,            // Stream mode - Circular buffer; overwrite old when full
    LIS3DSHTR_FIFO_STREAM_TO_FIFO = 0b011,    // Stream mode until INT2 de-asserted, then FIFO
    LIS3DSHTR_FIFO_BYPASS_TO_STREAM = 0b100,  // Bypass mode until INT2 de-asserted, then Stream
    LIS3DSHTR_FIFO_BYPASS_TO_FIFO = 0b111,    // Bypass mode until INT2 de-asserted, then FIFO
} lis3dshtr_fifo_mode_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t fifo_watermark_depth : 5;     // FIFO depth when watermark is enabled
        lis3dshtr_fifo_mode_t fifo_mode : 3;  // FIFO operation mode
    };
} lis3dshtr_fifo_control_t;

////////////////////////////////////////////////////////////////////////////////
// FIFO_SRC

typedef union {
    uint8_t raw;
    struct {
        uint8_t fifo_level : 5;        // Number of samples stored in FIFO - 1
        bool fifo_is_empty : 1;        // FIFO has no stored data
        bool fifo_is_full : 1;         // FIFO is completely full
        bool watermark_triggered : 1;  // FIFO is equal or higher than watermark level
    };
} lis3dshtr_fifo_status_t;

////////////////////////////////////////////////////////////////////////////////
// ST1_X, ST2_X

typedef enum LIS3DSHTR_OP_CODES {
    LIS3DSHTR_OP_NOP = 0x0,
    LIS3DSHTR_OP_TI1 = 0x1,
    LIS3DSHTR_OP_TI2 = 0x2,
    LIS3DSHTR_OP_TI3 = 0x3,
    LIS3DSHTR_OP_TI4 = 0x4,
    LIS3DSHTR_OP_GNTH1 = 0x5,
    LIS3DSHTR_OP_GNTH2 = 0x6,
    LIS3DSHTR_OP_LNTH1 = 0x7,
    LIS3DSHTR_OP_LNTH2 = 0x8,
    LIS3DSHTR_OP_GTTH1 = 0x9,
    LIS3DSHTR_OP_LLTH2 = 0xA,
    LIS3DSHTR_OP_GRTH1 = 0xB,
    LIS3DSHTR_OP_LRTH1 = 0xC,
    LIS3DSHTR_OP_GRTH2 = 0xD,
    LIS3DSHTR_OP_LRTH2 = 0xE,
    LIS3DSHTR_OP_NZERO = 0xF,
    LIS3DSHTR_OP_STOP = 0x00,
    LIS3DSHTR_OP_CONT = 0x11,
    LIS3DSHTR_OP_JMP = 0x22,
    LIS3DSHTR_OP_SRP = 0x33,
    LIS3DSHTR_OP_CRP = 0x44,
    LIS3DSHTR_OP_SETP = 0x55,
    LIS3DSHTR_OP_SETS1 = 0x66,
    LIS3DSHTR_OP_STHR1 = 0x77,
    LIS3DSHTR_OP_OUTC = 0x88,
    LIS3DSHTR_OP_OUTW = 0x99,
    LIS3DSHTR_OP_STHR2 = 0xAA,
    LIS3DSHTR_OP_DEC = 0xBB,
    LIS3DSHTR_OP_SISW = 0xCC,
    LIS3DSHTR_OP_REL = 0xDD,
    LIS3DSHTR_OP_STHR3 = 0xEE,
    LIS3DSHTR_OP_SSYNC = 0xFF,
    LIS3DSHTR_OP_SABS0 = 0x12,
    LIS3DSHTR_OP_SABS1 = 0x13,
    LIS3DSHTR_OP_SELMA = 0x14,
    LIS3DSHTR_OP_SRADI0 = 0x21,  // State machine 2 only
    LIS3DSHTR_OP_SRADI1 = 0x23,  // State machine 2 only
    LIS3DSHTR_OP_SELSA = 0x24,
    LIS3DSHTR_OP_SCS0 = 0x31,  // State machine 2 only
    LIS3DSHTR_OP_SCS1 = 0x32,  // State machine 2 only
    LIS3DSHTR_OP_SRTAM0 = 0x34,
    LIS3DSHTR_OP_STIM3 = 0x41,
    LIS3DSHTR_OP_STIM4 = 0x42,
    LIS3DSHTR_OP_SRTAM1 = 0x43
} lis3dshtr_op_code_t;

const uint8_t LIS3DSHTR_OP_CODE_REGISTER_SIZE = 16;
typedef struct {
    lis3dshtr_op_code_t op_codes[LIS3DSHTR_OP_CODE_REGISTER_SIZE];
} lis3dshtr_state_machine_code_register_t;

////////////////////////////////////////////////////////////////////////////////
// MASKn_A, MASKn_B
/**
 * Axis and sign mask (default) for State Machine motion-detection operation. For more
 * information refer to Section 7: "Axis mask filter"
 */

typedef enum LIS3DSHTR_MASKS {
    LIS3DSHTR_MASK_A,
    LIS3DSHTR_MASK_B,
} lis3dshtr_mask_selection_t;

typedef union {
    uint8_t raw;
    struct {
        bool v_negative_enabled : 1;
        bool v_positive_enabled : 1;
        bool z_negative_enabled : 1;
        bool z_positive_enabled : 1;
        bool y_negative_enabled : 1;
        bool y_positive_enabled : 1;
        bool x_negative_enabled : 1;
        bool x_positive_enabled : 1;
    };
} lis3dshtr_mask_t;

////////////////////////////////////////////////////////////////////////////////
// SETTn

typedef union {
    uint8_t raw;
    struct {
        bool stop_and_cont_trigger_interrupt_enabled : 1;
        bool always_evaluate_standard_mask_enabled : 1;
        bool threshold_3_enabled_for_mask_a : 1;
        bool constant_shift_enabled : 1;     // State machine 2 only.
        bool diff_data_enabled : 1;          // State machine 2 only
        bool signed_thresholds_enabled : 1;  // Enable negative thresholds for THRS1 and THRS2
        bool threshold_3_enabled_for_mask_b;
        bool peak_detection_enabled : 1;
    };
} lis3dshtr_state_machine_setting_t;

////////////////////////////////////////////////////////////////////////////////
// PRn

typedef union {
    uint8_t raw;
    struct {
        uint8_t reset_pointer : 4;
        uint8_t program_pointer : 4;
    };
} lis3dshtr_state_machine_pointers_t;

////////////////////////////////////////////////////////////////////////////////
// OUTSn

typedef union {
    uint8_t raw;
    struct {
        bool v_negative_shown : 1;
        bool v_positive_shown : 1;
        bool z_negative_shown : 1;
        bool z_positive_shown : 1;
        bool y_negative_shown : 1;
        bool y_positive_shown : 1;
        bool x_negative_shown : 1;
        bool x_positive_shown : 1;
    };
} lis3dshtr_state_machine_output_flags_t;

////////////////////////////////////////////////////////////////////////////////
// DES2

typedef uint8_t lis3dshtr_decimation_counter_t;

///////////////////////////////////////////////////////////////////////////////
// LIS3DSH Class

class LIS3DSHTR {
   public:
    // Config
    bool begin(uint8_t comm_mode = I2C_MODE, uint8_t address_or_cs = DEFAULT_I2C_ADDRESS);
    bool comms_working();

    bool write(lis3dshtr_offset_t input, lis3dshtr_axis_selection_t axis);
    bool write(lis3dshtr_constant_shift_t input, lis3dshtr_axis_selection_t axis);
    bool write(lis3dshtr_lc_t input);
    bool write(lis3dshtr_vector_filter_coeff_t input, lis3dshtr_vector_filter_coefficient_selection_t coefficient_index);
    bool write(lis3dshtr_power_config_t input);
    bool write(lis3dshtr_state_machine_config_t input, lis3dshtr_state_machine_selection_t sm_number);
    bool write(lis3dshtr_interrupt_config_t input);
    bool write(lis3dshtr_scale_config_t input);
    bool write(lis3dshtr_fifo_config_t input);
    bool write(lis3dshtr_fifo_control_t input);
    bool write(lis3dshtr_state_machine_code_register_t *input, lis3dshtr_state_machine_selection_t sm_number);
    bool write(lis3dshtr_state_machine_code_register_t input, lis3dshtr_state_machine_selection_t sm_number, uint8_t position);
    bool write(lis3dshtr_short_counter_t input, lis3dshtr_state_machine_selection_t sm_number);
    bool write(lis3dshtr_long_counter_t input, lis3dshtr_long_counter_selection_t counter_index);
    bool write(lis3dshtr_long_counter_t input, lis3dshtr_long_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number);
    bool write(lis3dshtr_thrs3_t input);
    bool write(lis3dshtr_threshold_t input, lis3dshtr_threshold_selection_t threshold_index);
    bool write(lis3dshtr_threshold_t input, lis3dshtr_threshold_selection_t threshold_index, lis3dshtr_state_machine_selection_t sm_number);
    bool write(lis3dshtr_mask_t input, lis3dshtr_mask_selection_t mask_index, lis3dshtr_state_machine_selection_t sm_number););
    bool write(lis3dshtr_state_machine_setting_t input, lis3dshtr_state_machine_selection_t sm_number);

    bool read(lis3dshtr_raw_temp_t &output);
    bool read(lis3dshtr_temp_t &output);
    bool read(lis3dshtr_info_t &output, uint8_t register_number);
    bool read(lis3dshtr_who_am_i_t &output);
    bool read(lis3dshtr_offset_t &output, lis3dshtr_axis_selection_t axis);
    bool read(lis3dshtr_constant_shift_t &output, lis3dshtr_axis_selection_t axis);
    bool read(lis3dshtr_long_counter_t &output);
    bool read(lis3dshtr_stat_t &output);
    bool read(lis3dshtr_peak_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_vector_filter_coeff_t &output, is3dshtr_vector_filter_coefficient_selection_t coefficient_index);
    bool read(lis3dshtr_power_config_t &output);
    bool read(lis3dshtr_state_machine_config_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_interrupt_config_t &output);
    bool read(lis3dshtr_scale_config_t &output);
    bool read(lis3dshtr_fifo_config_t &output);
    bool read(lis3dshtr_status_t &output);
    bool read(lis3dshtr_raw_data_t &output);
    bool read(lis3dshtr_roll_pitch_data_t &output);
    uin8_t read(lis3dshtr_fifo_data_t &output);
    bool read(lis3dshtr_fifo_control_t &output);
    bool read(lis3dshtr_state_machine_code_register_t *output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_state_machine_code_register_t *output, lis3dshtr_state_machine_selection_t sm_number, uint8_t position);
    bool read(lis3dshtr_fifo_status_t &output);
    bool read(lis3dshtr_short_counter_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_long_counter_t &output, lis3dshtr_long_counter_selection_t counter_index);
    bool read(lis3dshtr_long_counter_t &output, lis3dshtr_long_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_threshold_t &output);
    bool read(lis3dshtr_thrs3_t &output);
    bool read(lis3dshtr_mask_t &output, lis3dshtr_mask_selection_t mask_index, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_state_machine_setting_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_state_machine_pointers_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_state_machine_output_flags_t &output, lis3dshtr_state_machine_selection_t sm_number);
    bool read(lis3dshtr_decimation_counter_t &output);

   private:
    typedef enum LIS3DSHTR_REGISTER {
        OUT_T = 0x0C,
        INFO1 = 0x0D,
        INFO2 = 0x0E,
        WHO_AM_I = 0x0F,
        OFF_X = 0x10,
        OFF_Y = 0x11,
        OFF_Z = 0x12,
        CS_X = 0x13,
        CS_Y = 0x14,
        CS_Z = 0x15,
        LC_L = 0x16,
        LC_H = 0x17,
        STAT = 0x18,
        PEAK1 = 0x19,
        PEAK2 = 0x1A,
        VFC_1 = 0x1B,
        VFC_2 = 0x1C,
        VFC_3 = 0x1D,
        VFC_4 = 0x1E,
        THRS3 = 0x1F,
        CTRL_REG1 = 0x21,
        CTRL_REG2 = 0x22,
        CTRL_REG3 = 0x23,
        CTRL_REG4 = 0x20,
        CTRL_REG5 = 0x24,
        CTRL_REG6 = 0x25,
        STATUS = 0x27,
        X_L = 0x28,
        X_H = 0x29,
        Y_L = 0x2A,
        Y_H = 0x2B,
        Z_L = 0x2C,
        Z_H = 0x2D,
        FIFO_CTRL = 0x2E,
        FIFO_SRC = 0x2F,
        ST1_1 = 0x40,
        TIM4_1 = 0x50,
        TIM3_1 = 0x51,
        TIM2_1 = 0x52,  // 16-bit, Low byte first
        TIM1_1 = 0x54,  // 16-bit, Low byte first
        THRS2_1 = 0x56,
        THRS1_1 = 0x57,
        MASK1_B = 0x59,
        MASK1_A = 0x5A,
        SETT1 = 0x5B,
        PR1 = 0x5C,
        TC1 = 0x5D,    // 16-bit, Low byte first
        OUTS1 = 0x5F,  // 16-bit, Low byte first
        ST2_1 = 0x60,  // ST2_1 (60) ~ ST2_16 (6F)
        TIM4_2 = 0x70,
        TIM3_2 = 0x71,
        TIM2_2 = 0x72,  // 16-bit, Low byte first
        TIM1_2 = 0x74,  // 16-bit, Low byte first
        THRS2_2 = 0x76,
        THRS1_2 = 0x77,
        DES2 = 0x78,
        MASK2_B = 0x79,
        MASK2_A = 0x7A,
        SETT2 = 0x7B,
        PR2 = 0x7C,
        TC2 = 0x7D,    // 16-bit, Low byte first
        OUTS2 = 0x7F,  // 16-bit, Low byte first
    } lis3dshtr_register_t;

    bool write(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);
    bool read(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);
    bool write_i2c(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);
    bool read_i2c(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);
    bool write_spi(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);
    bool read_spi(uint8_t *input, lis3dshtr_register_t address, uint8_t length = 1);

    // Communication stuff
    uint8_t comm_type;
    uint8_t i2c_address_or_cs_pin;

    ///////////////////////////////////////////////////////////////////////////////
    // Device Registers
};

///////////////////////////////////////////////////////////////////////////////
// State Machine opcodes and commands

#endif
