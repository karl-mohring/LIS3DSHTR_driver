/******************************************************************************
SparkFunLIS3DH.cpp
LIS3DSH Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
Nov 16, 2016
https://github.com/sparkfun/LIS3DH_Breakout
https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

*/

#include "LIS3DSH.h"

#include "SPI.h"
#include "Wire.h"

///////////////////////////////////////////////////////////////////////////////
// Constructor

/**
 * Configure communication options for the accelerometer.
 * @param bus_type: I2C_MODE or SPI_MODE to indicate the type of comms used.
 * @param address_or_cs: I2C address or chip select pin, depending on comm type used.
 */
LIS3DSH::LIS3DSH(uint8_t bus_type, uint8_t address_or_cs) {
    if (bus_type == SPI_MODE) {
        comm_type = SPI_MODE;
        chip_select_pin = address_or_cs;
    }

    else {
        comm_type = I2C_MODE;
        i2c_address = address_or_cs;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Initialisation and config

/**
 * Prepare the device for use.
 * Communications are established and initial settings are applied.
 */
status_t LIS3DSH::begin(void) {
    status_t comm_result = begin_comms();
    apply_settings();

    return comm_result;
}

/**
 * Start up sensor communication.
 * @return: 1 == Success; 0 == Failure to read_from HW ID
 */
status_t LIS3DSH::begin_comms() {
    if (comm_type == SPI_MODE) {
        pinMode(chip_select_pin, OUTPUT);
        digitalWrite(chip_select_pin, HIGH);
        SPI.begin();

#if defined(ARDUINO_ARCH_ESP32)
        SPI.setFrequency(1000000);
        SPI.setBitOrder(SPI_MSBFIRST);
        // Like the standard arduino/teensy comment below, mode0 seems wrong according to standards
        // but conforms to the timing diagrams when used for the ESP32
        SPI.setDataMode(SPI_MODE0);

#elif defined(__MK20DX256__)

        // Maximum SPI frequency is 10MHz, could divide by 2 here:
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        SPI.setBitOrder(MSBFIRST);
        // Data is captured on rising edge of clock (CPHA = 0)
        // Base value of the clock is HIGH (CPOL = 1)
        // MODE0 for Teensy 3.1 operation
        SPI.setDataMode(SPI_MODE0);
#else
        // probably __AVR__
        // Maximum SPI frequency is 10MHz, could divide by 2 here:
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        SPI.setBitOrder(MSBFIRST);
        // Data is captured on rising edge of clock (CPHA = 0)
        // Base value of the clock is HIGH (CPOL = 1)
        // MODE3 for 328p operation
        SPI.setDataMode(SPI_MODE3);

#endif
    }
}

/**
 * Check that communications are working.
 * The hardware ID register is compared against its known value.
 * @return: 1 == The read ID matches the expected value, 0 == no match.
 */
status_t LIS3DSH::comms_check() {
    uint8_t read_check;
    status_t comm_result = IMU_HW_ERROR;

    read_from(&read_check, LIS3DSH_REG_WHO_AM_I);
    if (read_check == WHO_AM_I_ID) {
        comm_result = IMU_SUCCESS;
    }

    return comm_result;
}

/**
 * Write the device configuration to the device.
 * All settings should be configured before calling begin().
 * If any settings are changed, this method should be called again to update the device configuration.
 */
void LIS3DSH::apply_settings(void) {
    set_sample_rate();
    set_range_and_aa();
    configure_interrupts();
    configure_fifo();
    apply_state_machine_settings();
}

///////////////////////////////////////////////////////////////////////////////
// Comms

/**
 * Read data from the device registers.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
status_t LIS3DSH::read_from(uint8_t *output, uint8_t address, uint8_t length) {
    uint8_t result;
    status_t comm_result;

    if (comm_type == SPI_MODE) {
        comm_result = spi_read(output, address, length);
    } else {
        comm_result = i2c_read(output, address, length);
    }

    return comm_result;
}

/**
 * Read a specified number of bytes using the I2C bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
status_t LIS3DSH::i2c_read(uint8_t *output, uint8_t address, uint8_t length) {
    status_t result = IMU_SUCCESS;
    Wire.beginTransmission(i2c_address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = IMU_HW_ERROR;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(i2c_address, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

/**
 * Read a specified number of bytes using the SPI bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
status_t LIS3DSH::spi_read(uint8_t *output, uint8_t address, uint8_t length) {
    status_t result = IMU_SUCCESS;
    uint8_t num_empty_bytes = 0;

    digitalWrite(chip_select_pin, LOW);
    SPI.transfer(address | 0x80 | 0x40);
    for (size_t i = 0; i < length; i++) {
        uint8_t c = SPI.transfer(0x00);
        if (c == 0xFF) num_empty_bytes++;
        *output = c;
        output++;
    }
    if (num_empty_bytes == length) result = IMU_ALL_ONES_WARNING;
    digitalWrite(chip_select_pin, HIGH);

    return result;
}

/**
 * Write a value to a register.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::write_to(uint8_t *input, uint8_t address, uint8_t length) {
    uint8_t result;
    status_t comm_result;

    if (comm_type == SPI_MODE) {
        comm_result = spi_write(input, address);
    } else {
        comm_result = i2c_write(input, address);
    }

    return comm_result;
}

/**
 * Write a value to a register using I2C
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::i2c_write(uint8_t *input, uint8_t address, uint8_t length) {
    status_t result = IMU_SUCCESS;
    Wire.beginTransmission(i2c_address);
    Wire.write(address);
    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        result = IMU_HW_ERROR;
    }
    return result;
}

/**
 * Write a value to a register using SPI
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::spi_write(uint8_t *input, uint8_t address, uint8_t length) {
    digitalWrite(chip_select_pin, LOW);
    SPI.transfer(address);
    for (size_t i = 0; i < length; i++) {
        SPI.transfer(input[i]);
    }
    digitalWrite(chip_select_pin, HIGH);
    return IMU_SUCCESS;
}

/**
 * Write a value to a single bit within a register.
 *
 * @param input: Byte to write to the register.
 * @param address: Address of register to write to.
 * @param bit: Bit address to be written within register.
 * @return: Success/error result of the write.
 */
status_t LIS3DSH::write_bit(uint8_t input, uint8_t address, uint8_t bit) {
    status_t comm_result = IMU_SUCCESS;
    uint8_t current_state;

    // Ensure input is a 1-bit flag
    input &= 0x01;

    comm_result = read_from(&current_state, address);
    bitWrite(current_state, bit, input);
    comm_result = write_to(&current_state, address);

    return comm_result;
}

///////////////////////////////////////////////////////////////////////////////
// Config

/**
 * Set the sample rate (ODR) of the accelerometer.
 * Valid options are [0, (power down), 3, 6, 12, 25, 50, 100, 400, 800, 1600].
 * Invalid options will put the accelerometer into power down mode.
 *
 * Sample rate can be changed in the device settings.
 */
void LIS3DSH::set_sample_rate() {
    uint8_t input = 0;
    switch (settings.sample_rate) {
        case 3:
            input |= (0x01 << 4);
            break;
        case 6:
            input |= (0x02 << 4);
            break;
        case 12:
            input |= (0x03 << 4);
            break;
        case 25:
            input |= (0x04 << 4);
            break;
        case 50:
            input |= (0x05 << 4);
            break;
        case 100:
            input |= (0x06 << 4);
            break;
        case 400:
            input |= (0x07 << 4);
            break;
        case 800:
            input |= (0x08 << 4);
            break;
        case 1600:
            input |= (0x09 << 4);
            break;
        default:
            input = 0;  // Power down
            break;
    }

    input |= (settings.block_data_update & 0x1) << 3;
    input |= (settings.accelerometer_x_enabled & 0x1) << 2;
    input |= (settings.accelerometer_y_enabled & 0x1) << 1;
    input |= (settings.accelerometer_z_enabled & 0x1);
    write_to(&input, LIS3DSH_CTRL_REG4);
}

/**
 * Set the full scale range and the anti-aliasing filter bandwidth of the device.
 * Valid ranges are [2, 4, 6, 8, 16] g.
 * Valid AA bandwidths are [50, 100, 400, 800] Hz.
 */
void LIS3DSH::set_range_and_aa() {
    uint8_t input = 0;
    uint8_t setting;

    // AA filter bandwidth
    switch (settings.aa_filter_bandwidth) {
        case 50:
            setting = 0b11;
            break;

        case 200:
            setting = 0b01;
            break;

        case 400:
            setting = 0b10;
            break;

        default:
        case 800:
            setting = 0b00;
            break;
    }
    input |= (setting << 6);

    // Range
    switch (settings.accelerometer_range) {
        default:
        case 2:
            setting = 0b000;
            break;
        case 4:
            setting = 0b001;
            break;
        case 6:
            setting = 0b010;
            break;
        case 8:
            setting = 0b011;
            break;
        case 16:
            setting = 0b100;
            break;
    }
    input |= (setting << 3);

    // Now, write the patched together data
    write_to(&input, LIS3DSH_CTRL_REG5);
}

/**
 * Apply settings for the first-in;first-out buffer
 */
void LIS3DSH::configure_fifo() {
    // Constrain some settings
    settings.fifo_enabled = bool(settings.fifo_enabled);
    settings.fifo_watermark_interrupt_enabled = bool(settings.fifo_watermark_interrupt_enabled);
    uint8_t fifo_mode = constrain(settings.fifo_mode, 0, 0b111);
    uint8_t watermark = constrain(settings.fifo_watermark, 0, 32);

    // Enable/disable fifo - CTRL_REG6
    uint8_t register_value = 0;
    bitSet(register_value, 4);  // Enabled ADD_INC - automatic address increment during burst read_from
    bitWrite(register_value, 6, settings.fifo_enabled);
    _is_fifo_active = settings.fifo_enabled;
    bitWrite(register_value, 2, settings.fifo_watermark_interrupt_enabled);
    write_to(&register_value, LIS3DSH_CTRL_REG6);

    // Set up FIFO options - FIFO_CTRL
    register_value = (fifo_mode << 5) | watermark;
    write_to(&register_value, LIS3DSH_REG_FIFO_CTRL);
}

/**
 * Configure interrupts.
 * Interrupts are enabled/disabled, as well as their polarity and output type.
 */
void LIS3DSH::configure_interrupts() {
    uint8_t interrupt_level = bool(settings.interrupt_polarity);
    uint8_t interrupt_latch = bool(settings.interrupt_latching);
    uint8_t interrupt_1 = bool(settings.interrupt_1_enabled);
    uint8_t interrupt_2 = bool(settings.interrupt_2_enabled);
    uint8_t ready_interrupt = bool(settings.data_ready_interrupt_enabled);

    uint8_t register_value = 0;
    bitWrite(register_value, 7, ready_interrupt);
    bitWrite(register_value, 6, interrupt_level);
    bitWrite(register_value, 5, interrupt_latch);
    bitWrite(register_value, 4, interrupt_2);
    bitWrite(register_value, 3, interrupt_1);

    if (settings.fifo_watermark_interrupt_enabled) {
        bitSet(register_value, 3);
    }
    write_to(&register_value, LIS3DSH_CTRL_REG3);
}

/**
 * Apply settings to the state machines.
 * State machine settings are only written if the state machine is enabled in the main device settings.
 */
void LIS3DSH::apply_state_machine_settings() {
    if (settings.state_machine_1_enabled) {
        configure_state_machine(SM1);
    }

    if (settings.state_machine_2_enabled) {
        configure_state_machine(SM2);
    }
}

/**
 * Apply settings for the specified state machine.
 *
 * @param sm_number: State machine to be configured.
 */
void LIS3DSH::configure_state_machine(uint8_t sm_number) {
    write_state_machine_status(sm_number, false);

    uint8_t offset = 0;
    if (sm_number == SM2) offset = 0x20;

    // CTRL_REG[1,2]
    uint8_t register_value = 0;
    register_value |= constrain(sm_settings[sm_number].hysteresis, 0, 7) << 5;
    register_value |= constrain(sm_settings[sm_number].interrupt_output, 0, 1) << 3;
    write_to(&register_value, LIS3DSH_CTRL_REG1 + offset);

    // SM code registers
    for (size_t i = 0; i < 16; i++) {
        uint8_t code = sm_settings[sm_number].code[i];
        set_state_machine_instruction(sm_number, i + 1, code);
    }

    // Thresholds
    write_to(&sm_settings[sm_number].threshold_1, LIS3DSH_REG_THRS1_1 + offset);
    write_to(&sm_settings[sm_number].threshold_2, LIS3DSH_REG_THRS2_1 + offset);

    // Timers
    uint8_t timer[2] = {lowByte(sm_settings[sm_number].timer1_initial_value), highByte(sm_settings[sm_number].timer1_initial_value)};
    write_to(timer, LIS3DSH_REG_TIM1_1 + offset, 2);

    timer[0] = lowByte(sm_settings[sm_number].timer2_initial_value);
    timer[1] = highByte(sm_settings[sm_number].timer2_initial_value);
    write_to(timer, LIS3DSH_REG_TIM2_1 + offset, 2);

    write_to(&sm_settings[sm_number].timer3_initial_value, LIS3DSH_REG_TIM3_1 + offset);
    write_to(&sm_settings[sm_number].timer4_initial_value, LIS3DSH_REG_TIM4_1 + offset);

    // SM SETT
    register_value = 0;
    register_value |= ((sm_settings[sm_number].thresholds_are_absolute & 0x1) << 5);
    if (sm_number == SM2) {
        register_value |= ((sm_settings[SM2].diff_calculation_enabled & 0x1) << 4);
        register_value |= ((sm_settings[SM2].diff_from_constants_enabled & 0x1) << 3);
    }
    register_value |= (sm_settings[sm_number].stop_and_cont_interrupts & 0x1);
    write_to(&register_value, LIS3DSH_REG_SETT1 + offset);

    // Masks
    write_to(&sm_settings[sm_number].mask_a, LIS3DSH_REG_MASK1_A + offset);
    write_to(&sm_settings[sm_number].mask_b, LIS3DSH_REG_MASK1_B + offset);

    // SM2 Only
    if (sm_number == SM2) {
        write_to(&sm_settings[sm_number].decimator, LIS3DSH_REG_DES2);
    }

    write_state_machine_status(sm_number, true);
}

/**
 * Stop the device from sampling.
 * Data will stop being sampled entirely.
 * The FIFO buffer will not fill.
 */
void LIS3DSH::power_down() {
    uint8_t i = 0;
    write_to(&i, LIS3DSH_CTRL_REG4);
}

/**
 * Set the device to record measurements after previously being placed in power down mode.
 * The sampling frequency will return to what has been set in the device settings.
 */
void LIS3DSH::measurement_mode() { set_sample_rate(); }

/**
 * Reboot the device.
 * Might take a little while?
 */
void LIS3DSH::reboot() {
    uint8_t boot = 0x80;
    write_to(&boot, LIS3DSH_CTRL_REG6);
}
///////////////////////////////////////////////////////////////////////////////
// Outputs

/**
 * Read the temperature from the device.
 * @return: Temperature in deg C (no decimal precision).
 */
int8_t LIS3DSH::read_temperature() {
    uint8_t temperature;
    read_from(&temperature, LIS3DSH_REG_OUT_T);
    temperature += 25;
    return temperature;
}

/**
 * Calculate the acceleration from a raw value.
 * @param input: Raw acceleration as read from the device's output register.
 * @return: Acceleration of the given measurement in g.
 */
float LIS3DSH::calculate_acceleration_from_raw(int16_t input) {
    float output;
    uint8_t range = constrain(settings.accelerometer_range, 2, 16);
    float divisor = 0x7FFF / settings.accelerometer_range;
    return float(output / divisor);
}

/**
 * Read the accelerometer measurements.
 * Measurements are in their raw form.
 * If FIFO is enabled, the measurement returned is the oldest in the buffer.
 *
 * @param readings: 3xInt buffer to store readings into (X,Y,Z).
 */
void LIS3DSH::read_accelerometers(int16_t *readings) {
    uint8_t data[6];
    read_from(data, LIS3DSH_REG_X_L, 6);

    for (size_t i = 0; i < 3; i++) {
        readings[i] = (data[i * 2] | (data[i * 2 + 1] << 8));
    }
}

/**
 * Read the accelerometer measurements.
 * Measurements are in their raw form.
 * If FIFO is enabled, the measurement returned is the oldest in the buffer.
 *
 * @param entry: Accelerometer entry object to store the measurements.
 */
void LIS3DSH::read_accelerometers(AccelerometerEntry *entry) {
    int16_t data[3];
    read_accelerometers(data);
    entry->x = data[0];
    entry->y = data[1];
    entry->z = data[2];
}

///////////////////////////////////////////////////////////////////////////////
// FIFO

void LIS3DSH::restore_fifo() { configure_fifo(); }

void LIS3DSH::disable_fifo() {
    write_bit(0, LIS3DSH_CTRL_REG6, 6);  // Disable FIFO
    byte zero = 0;
    write_to(&zero, LIS3DSH_REG_FIFO_CTRL);
    _is_fifo_active = false;
}

/**
 * Check if the fifo has been enabled or disabled.
 * @return: Fifo active state. true=active
 */
uint8_t LIS3DSH::is_fifo_active() { return _is_fifo_active; }

/**
 * Read in the contents of the FIFO buffer.
 * A maximum of 32 measurements can be read in at once.
 * Oldest measurements are collected first.
 * Ensure the specified output buffer has enough space to fit the entire buffer (max. 192 bytes).
 *
 * @param output_buffer: Output buffer to store the collected data.
 * @return: The number of measurements read_from from the FIFO buffer.
 */
uint8_t LIS3DSH::read_fifo_buffer(uint8_t *output_buffer) {
    uint8_t entries_to_read = get_fifo_count();

    if (entries_to_read > 0) {
        uint8_t bytes_to_read = constrain(entries_to_read * 6, 0, 192);
        read_from(output_buffer, LIS3DSH_REG_X_L, bytes_to_read);
    }

    return entries_to_read;
}

/**
 * Read in the contents of the FIFO buffer.
 * A maximum of 32 measurements can be read in at once.
 * Oldest measurements are collected first.
 * Ensure the specified output buffer has enough space to fit the entire buffer (max. 32 entries).
 *
 * @param output_buffer: Output buffer to store the collected data.
 * @return: The number of measurements read_from from the FIFO buffer.
 */
uint8_t LIS3DSH::read_fifo_buffer(AccelerometerEntry *buffer) {
    uint8_t data[192];
    uint8_t num_entries = read_fifo_buffer(data);

    for (size_t i = 0; i < num_entries; i++) {
        buffer[i].x = data[i * 6] + (data[i * 6 + 1] << 8);
        buffer[i].y = data[i * 6 + 2] + (data[i * 6 + 3] << 8);
        buffer[i].z = data[i * 6 + 4] + (data[i * 6 + 5] << 8);
    }
    return num_entries;
}

/**
 * Get the number of measurements currently stored in the FIFO buffer.
 * @return: Number of stored measurements - 1.
 */
uint8_t LIS3DSH::get_fifo_count() {
    uint8_t fifo_state;
    read_from(&fifo_state, LIS3DSH_REG_FIFO_SRC);
    uint8_t entries_in_fifo = (fifo_state & 0b11111);
    return entries_in_fifo;
}

/**
 * Check whether measurements in the FIFO buffer have been overwritten.
 * Overwrites can occur when the FIFO is read at a slower rate than the incoming measurements.
 * @return: True if overwritten
 */
uint8_t LIS3DSH::has_fifo_overrun() {
    uint8_t fifo_state;
    read_from(&fifo_state, LIS3DSH_REG_FIFO_SRC);
    return bitRead(fifo_state, 6);
}

///////////////////////////////////////////////////////////////////////////////
// State machines

/**
 * Write a code instruction for the specified state machine state.
 *
 * @param sm_number: The state machine to write instructions to [SM1 or SM2].
 * @param code_register_id: The state to write the instruction to [1 - 16].
 * @param instruction: An 8-bit value containing a command or two op codes in [RESET, NEXT] format.
 */
void LIS3DSH::set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t instruction) {
    // Make sure the inputs are in range
    sm_number = constrain(sm_number, SM1, SM2);
    code_register_id = constrain(code_register_id, 1, 16);

    // Calculate the correct register address
    uint8_t register_address = LIS3DSH_REG_ST1_1;
    if (sm_number == SM2) {
        register_address = LIS3DSH_REG_ST2_1;
    }
    register_address += (code_register_id - 1);

    write_to(&instruction, register_address);
}

/**
 * Write a code instruction for the specified state machine state.
 *
 * @param sm_number: The state machine to write instructions to [SM1 or SM2].
 * @param code_register_id: The state to write the instruction to [1 - 16].
 * @param reset_instruction: The opcode to specify a condition to reset the state machine to its last reset position.
 * @param next_instruction: The opcode to specify a condition to progress the state machine into the next state.
 */
void LIS3DSH::set_state_machine_instruction(uint8_t sm_number, uint8_t code_register_id, uint8_t reset_instruction, uint8_t next_instruction) {
    reset_instruction &= 0x0F;
    next_instruction &= 0x0F;
    uint8_t instruction = (reset_instruction << 4) + next_instruction;
    set_state_machine_instruction(sm_number, code_register_id, instruction);
}

/**
 * Enable or disable the specified state machine.
 * @param sm_number: The state machine to set [SM1 or SM2]
 * @param active_status: Resulting status of the state machine [1: enabled, 0: disabled]
 */
void LIS3DSH::write_state_machine_status(uint8_t sm_number, uint8_t active_status) {
    uint8_t register_address = LIS3DSH_CTRL_REG1;
    if (sm_number == SM2) register_address = LIS3DSH_CTRL_REG2;

    write_bit(active_status, register_address, 0);
}

/**
 * Configure the device to only record on activity.
 * This configuration changes the following settings:
 * - The FIFO buffer will only fill after activity exceeds a minimum differential threshold between readings.
 * - Measurements will stop being recorded after a minimum period of 20 seconds after activity has fallen below a second
 *      minimum differential threshold.
 */
void LIS3DSH::configure_auto_sleep() {
    settings.state_machine_2_enabled = true;

    StateMachineSettings sleep_settings;

    sleep_settings.decimator = 4;
    sleep_settings.threshold_1 = settings.wake_threshold;
    sleep_settings.threshold_2 = settings.sleep_threshold;
    sleep_settings.thresholds_are_absolute = true;
    sleep_settings.diff_calculation_enabled = true;
    sleep_settings.stop_and_cont_interrupts = settings.interrupt_on_inactivity;
    sleep_settings.interrupt_output = settings.activity_interrupt_pin;

    sleep_settings.mask_a = 0b11111100;
    sleep_settings.mask_b = 0b11111100;

    /**
     * [Inactive]   - Wait for the activity to go above the wake threshold (TH1)
     *              - Activity moves the routine into the active state.
     *
     * [Active]     - Start recording measurements to FIFO (if in a triggered mode)
     *              - Loop back to 2 while activity stays above the inactive threshold (TH2)
     *              - If activity drops below the inactive threshold, the device moves to the pre-sleep stage
     */

    // Inactive state
    sleep_settings.code[0] = (LIS3DSH_OP_NOP << 4) | LIS3DSH_OP_GNTH1;  // Wait until wake threshold triggered

    // Active state
    sleep_settings.code[1] = LIS3DSH_COMMAND_OUTC;                      // Trigger interrupt to prompt MCU to enable FIFO
    sleep_settings.code[2] = (LIS3DSH_OP_NOP << 4) | LIS3DSH_OP_LLTH2;  // Hold while activity continues on any axis
    sleep_settings.code[3] = LIS3DSH_COMMAND_CONT;                      // Activity dropped below threshold; reset

    sm_settings[SM2] = sleep_settings;
    apply_state_machine_settings();
}

/**
 * Get the active state of the specified state machine
 * @return: Pointer address of the active state [0-15]
 */
uint8_t LIS3DSH::get_state(uint8_t sm_number) {
    uint8_t state;
    uint8_t register_address = LIS3DSH_REG_PR1;
    if (sm_number == SM2) register_address = LIS3DSH_REG_PR2;

    read_from(&state, register_address);
    return (state & 0x0F);
}

uint8_t LIS3DSH::read_state_machine_output(uint8_t sm_number) {
    uint8_t address = LIS3DSH_REG_OUTS1;
    if (sm_number == SM2) address = LIS3DSH_REG_OUTS2;

    uint8_t output;
    read_from(&output, address);
    return output;
}