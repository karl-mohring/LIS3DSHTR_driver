#include <LIS3DSHTR.h>

////////////////////////////////////////////////////////////////////////////////
// Initialisation

/**
 * Start communication with the LIS3DSHTR accelerometer.
 *
 * The accelerometer can use either SPI or I²C communication.
 * Communication type is selected by applying a HIGH or LOW signal on the CS pin (8).
 * A logical HIGH enables I²C, while a logical LOW enables SPI.
 *
 * @param comm_mode Communication mode used with the sensor. Depends on physical configuration.
 * @param address_or_cs The address of the accelerometer if using I²C mode, or the controller's CS pin if using SPI.
 * @return True if communication could be started and the device ID was read successfully.
 */
bool LIS3DSHTR::begin(uint8_t comm_mode, uint8_t address_or_cs) {
    this->comms_type = comm_mode;
    this->i2c_address_or_cs_pin = address_or_cs;
    return comms_working();
}

/**
 * Test communication with the accelerometer.
 *
 * The device's ID is read and matched against the known ID.
 * An improperly configured or disconnected device will result in a mismatched ID.
 *
 * @return True if the read ID matches the expected value.
 */
bool LIS3DSHTR::comms_working() {
    lis3dshtr_who_am_i_t device;
    read(device);

    return device.id == LIS3DSHTR_WHO_AM_I_ID;
}

////////////////////////////////////////////////////////////////////////////////
// Generic communication methods

/**
 * Generic write to the device.
 *
 * The write function selects the appropriate write method, based on the chosen communication type.
 * Multiple byte reads may need automatic incrementation to be enabled.
 *
 * @param input Pointer to the data to be written to the device.
 * @param address Register address to which data will be written.
 * @param length Number of bytes to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(uint8_t *input, lis3dshtr_register_t address, uint8_t length) {
    bool success = false;
    if (this->comms_type == LIS3DSHTR_I2C_MODE) success = write_i2c(input, address, length);
    if (this->comms_type == LIS3DSHTR_SPI_MODE) success = write_spi(input, address, length);
    return success;
}

/**
 * Generic write to the device using I²C.
 *
 * @param input Pointer to the data to be written to the device.
 * @param address Register address to which data will be written.
 * @param length Number of bytes to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write_i2c(uint8_t *input, lis3dshtr_register_t address, uint8_t length) {
    bool success = true;
    while (Wire.available()) Wire.read();  // Flush the bus
    Wire.beginTransmission(this->i2c_address_or_cs_pin);
    Wire.write(address);
    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        success = false;
    }
    return success;
}

/**
 * Generic write to the device using SPI.
 *
 * @param input Pointer to the data to be written to the device.
 * @param address Register address to which data will be written.
 * @param length Number of bytes to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write_spi(uint8_t *input, lis3dshtr_register_t address, uint8_t length) {
    bool success = false;

    if (this->i2c_address_or_cs_pin > 0) {
        digitalWrite(this->i2c_address_or_cs_pin, LOW);
        SPI.transfer(address);
        for (size_t i = 0; i < length; i++) {
            SPI.transfer(input[i]);
        }
        digitalWrite(this->i2c_address_or_cs_pin, HIGH);
        success = true;
    }

    return success;
}

/**
 * Generic read from the device.
 *
 * The write function selects the appropriate write method, based on the chosen communication type.
 * Multiple byte reads may need automatic incrementation to be enabled.
 *
 * @param input Pointer to the container in which read data will be placed.
 * @param address Register address from which data will be read.
 * @param length Number of bytes to be read.
 * @return True if the read was successful.
 */
bool LIS3DSHTR::read(uint8_t *output, lis3dshtr_register_t address, uint8_t length) {
    bool success = false;
    if (this->comms_type == LIS3DSHTR_I2C_MODE) success = read_i2c(output, address, length);
    if (this->comms_type == LIS3DSHTR_SPI_MODE) success = read_spi(output, address, length);
    return success;
}

/**
 * Generic read from the device using I²C.
 *
 * @param input Pointer to the container in which read data will be placed.
 * @param address Register address from which data will be read.
 * @param length Number of bytes to be read.
 * @return True if the read was successful.
 */
bool LIS3DSHTR::read_i2c(uint8_t *output, lis3dshtr_register_t address, uint8_t length) {
    bool success = false;
    Wire.beginTransmission(this->i2c_address_or_cs_pin);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        success = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(uint8_t(this->i2c_address_or_cs_pin), length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
            if ((i + 1) == length) success = true;
        }
    }
    return success;
}

/**
 * Generic read from the device using SPI.
 *
 * @param input Pointer to the container in which read data will be placed.
 * @param address Register address from which data will be read.
 * @param length Number of bytes to be read.
 * @return True if the read was successful.
 */
bool LIS3DSHTR::read_spi(uint8_t *output, lis3dshtr_register_t address, uint8_t length) {
    bool success = false;
    uint8_t num_empty_bytes = 0;

    // Proceed if a valid pin has been selected
    if (this->i2c_address_or_cs_pin > 0) {
        digitalWrite(this->i2c_address_or_cs_pin, LOW);
        SPI.transfer(this->i2c_address_or_cs_pin | 0x80 | 0x40);
        for (size_t i = 0; i < length; i++) {
            uint8_t c = SPI.transfer(0x00);
            if (c == 0xFF) num_empty_bytes++;
            *output = c;
            output++;
        }
        if (num_empty_bytes != length) success = true;
        digitalWrite(this->i2c_address_or_cs_pin, HIGH);
    }

    return success;
}

////////////////////////////////////////////////////////////////////////////////
// Write methods

/**
 * Write an offset correction to the device (OFF_x).
 *
 * The write will fail if an invalid axis is supplied.
 * Offsets are written on a per-axis basis only.
 * Three separate writes are required to apply an offset to each axis.
 *
 * @param input Offset correction value (signed).
 * @param axis Axis to which the correction is applied (LIS3DSHTR_[X,Y,Z]_AXIS).
 * @return True if write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_offset_t input, lis3dshtr_axis_selection_t axis) {
    bool success = false;
    if (axis == LIS3DSHTR_X_AXIS)
        success = write((uint8_t *)&input, OFF_X);
    else if (axis == LIS3DSHTR_Y_AXIS)
        success = write((uint8_t *)&input, OFF_Y);
    else if (axis == LIS3DSHTR_Z_AXIS)
        success = write((uint8_t *)&input, OFF_Z);
    return success;
}

/**
 * Write a constant shift value to the device (CS_x).
 *
 * Constant shifts are temporary offsets that are used in state machine operations.
 * The write will fail if an invalid axis is supplied.
 * Constant shifts are written on a per-axis basis only.
 * Three separate writes are required to apply a constant shift to each axis.
 *
 * @param input Offset correction value (signed).
 * @param axis Axis to which the correction is applied (LIS3DSHTR_[X,Y,Z]_AXIS).
 * @return True if write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_constant_shift_t input, lis3dshtr_axis_selection_t axis) {
    bool success = false;
    if (axis == LIS3DSHTR_X_AXIS)
        success = write((uint8_t *)&input, CS_X);
    else if (axis == LIS3DSHTR_Y_AXIS)
        success = write((uint8_t *)&input, CS_Y);
    else if (axis == LIS3DSHTR_Z_AXIS)
        success = write((uint8_t *)&input, CS_Z);
    return success;
}

/**
 * Write a value to the device's long counter (LC).
 *
 * The 16-bit long counter is used for interrupt state machine program timing.
 *
 * @param input Long counter value to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_lc_t input) { return write((uint8_t *)&input, LC_L, 2); }

/**
 * Write a coefficient for the vector filter (VFC_n).
 *
 * The vector filter is a 7th-order anti-symmetric FIR filter with the following transfer function:
 * Xv_filt = (x0- x7) coeff0 + (x1-x6) coeff1+ (x2-x5) coeff2 + (x3-x4) coeff3
 * The vector filter must be enabled to be used.
 * The write will fail if a valid coefficient index is not supplied.
 *
 * @see lis3dshtr_interrupt_config_t
 *
 * @param input Coefficient value to be written (signed).
 * @param coefficient_index Index of the coefficient to be written (LIS3DSHTR_VFC_[1-4] or LIS3DSHTR_VECTOR_COEFF_[0-3]).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_vector_filter_coeff_t input, lis3dshtr_vector_filter_coefficient_selection_t coefficient_index) {
    bool success = false;
    if (coefficient_index == LIS3DSHTR_VFC_1)
        success = write((uint8_t *)&input, VFC_1);
    else if (coefficient_index == LIS3DSHTR_VFC_2)
        success = write((uint8_t *)&input, VFC_2);
    else if (coefficient_index == LIS3DSHTR_VFC_3)
        success = write((uint8_t *)&input, VFC_3);
    else if (coefficient_index == LIS3DSHTR_VFC_4)
        success = write((uint8_t *)&input, VFC_4);

    return success;
}

/**
 * Write to the device's power and sampling register (CTRL_REG4).
 *
 * @param input Configuration to be written to the device.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_power_config_t input) { return write((uint8_t *)&input, CTRL_REG4); }

/**
 * Write a configuration to one of the device's state machines (CTRL_REG1, CTRL_REG2).
 *
 * @param input Configuration to be written to the state machine.
 * @param sm_number State machine to which the configuration is applied (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_state_machine_config_t input, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = write((uint8_t *)&input, CTRL_REG1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = write((uint8_t *)&input, CTRL_REG2);

    return success;
}

/**
 * Write a configuration to the interrupt control register (CTRL_REG3).
 *
 * @param input Configuration to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_interrupt_config_t input) { return write((uint8_t *)&input, CTRL_REG3); }

/**
 * Write a configuration to the scale configuration register (CTRL_REG5)
 *
 * @param input Configuration to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_scale_config_t input) { return write((uint8_t *)&input, CTRL_REG5); }

/**
 * Write to the FIFO configuration register (CTRL_REG6)
 *
 * @param input Configuration to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_fifo_config_t input) { return write((uint8_t *)&input, CTRL_REG6); }

/**
 * Write a configuration to FIFO control register (FIFO_CTRL)
 *
 * @param input Configuration to be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_fifo_control_t input) { return write((uint8_t *)&input, FIFO_CTRL); }

/**
 * Write an instruction set to the state machine code registers (STm_n).
 *
 * All registers are written at once.
 * Any existing information in the code registers will be overwritten.
 *
 * @param input Pointer to the buffer containing the op codes to be written.
 * @param sm_number State machine to which the op codes will be written (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_state_machine_code_register_t *input, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = write((uint8_t *)input, ST1_1, LIS3DSHTR_OP_CODE_REGISTER_SIZE);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = write((uint8_t *)input, ST2_1, LIS3DSHTR_OP_CODE_REGISTER_SIZE);
    return success;
}

/**
 * Write a single op code to one of the state machines' code registers (STm_n).
 *
 * @param input Op code to be written.
 * @param sm_number State machine to which the op code will be written.
 * @param position Position to which the op code will be written in the state machine register (0-15)
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_state_machine_code_register_t input, lis3dshtr_state_machine_selection_t sm_number, uint8_t position) {
    bool success = false;
    if (position < LIS3DSHTR_OP_CODE_REGISTER_SIZE) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
            success = write((uint8_t *)&input, lis3dshtr_register_t(ST1_1 + position));
        else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
            success = write((uint8_t *)&input, lis3dshtr_register_t(ST2_1 + position));
    }
    return success;
}

/**
 * Write a value to a short counter (TIM3_m, TIM4_m).
 *
 * An invalid counter index or state machine number will cause the write to fail.
 *
 * @param input Value to be written to the short counter.
 * @param counter_index Index of the counter to which the value is written (LIS3DSHTR_TIM[3, 4])
 * @param sm_number State machine number of the counter to be written (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_short_counter_t input, lis3dshtr_short_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (counter_index == LIS3DSHTR_TIM4)
        address = TIM4_1;
    else if (counter_index == LIS3DSHTR_TIM3)
        address = TIM3_1;

    if (address != 0) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
            address += (TIM4_2 - TIM4_1);
        }
        success = write((uint8_t *)&input, (lis3dshtr_register_t)address);
    }

    return success;
}

/**
 * Write a value to a long counter.
 *
 * This function is only used to write to the shared long counter, as it's the only long counter without a state machine number.
 * It's a redundant method, but its existance allows a consistent writing style to all long counters.
 *
 * @param input Value to write to the long counter.
 * @param counter_index Index of the counter to which the value will be written (LIS3DSHTR_LC).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_long_counter_t input, lis3dshtr_long_counter_selection_t counter_index) {
    bool success = false;
    if (counter_index == LIS3DSHTR_LC) success = write((lis3dshtr_lc_t)input);
    return success;
}

/**
 * Write a value to a long counter.
 *
 * @param input Value to write to the long counter.
 * @param counter_index Index of the counter to which the value will be written (LIS3DSHTR_[LC, TIM1, TIM2])
 * @param sm_number State machine to which the long counter is written (LIS3DSHTR_STATE_MACHINE_[1,2]). Does not apply to LIS3DSHTR_LC.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_long_counter_t input, lis3dshtr_long_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (counter_index == LIS3DSHTR_LC)
        success = write((lis3dshtr_lc_t)input);

    else {
        if (counter_index == LIS3DSHTR_TC)
            address = TC1;
        else if (counter_index == LIS3DSHTR_TIM1)
            address = TIM1_1;
        else if (counter_index == LIS3DSHTR_TIM2)
            address = TIM2_1;

        if (address != 0) {
            if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
                address += (TC2 - TC1);
            }
            success = write((uint8_t *)&input, (lis3dshtr_register_t)address, 2);
        }
    }

    return success;
}

/**
 * Write a value to the shared threshold (THRS3).
 *
 * @param input Value to be written to THRS3.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_thrs3_t input) { return write((uint8_t *)&input, THRS3); }

/**
 * Write a value to a threshold.
 *
 * This function is only used to write to the shared threshold (THRS3), as it's the only threshold without a state machine number.
 * It's a redundant method, but its existance allows a consistent writing style to all thresholds.
 *
 * @param input Value to be written to the threshold.
 * @param threshold_index Index of the threshold to which the value will be written (LIS3DSHTR_THRS3)
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_threshold_t input, lis3dshtr_threshold_selection_t threshold_index) {
    bool success = false;
    lis3dshtr_thrs3_t thresh;
    thresh.raw = input;
    if (threshold_index == LIS3DSHTR_THRS3) success = write(thresh);
    return success;
}

/**
 * Write a value to a threshold.
 *
 * @param input Value to be written to the threshold.
 * @param threshold_index Index of the threshold to which the value will be written (LIS3DSHTR_THRS[1,2,3])
 * @param sm_number State machine to which the threshold is written (LIS3DSHTR_STATE_MACHINE_[1,2]). Does not apply to LIS3DSHTR_THRS3.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_threshold_t input, lis3dshtr_threshold_selection_t threshold_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (threshold_index == LIS3DSHTR_THRS3)
        success = write(input, threshold_index);
    else {
        if (threshold_index == LIS3DSHTR_THRS1) address = THRS1_1;
        if (threshold_index == LIS3DSHTR_THRS2) address = THRS2_1;

        if (address != 0) {
            if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
                address += (THRS1_2 - THRS1_1);
            }
            success = write((uint8_t *)&input, (lis3dshtr_register_t)address);
        }
    }
    return success;
}

/**
 * Write a mask configuration to a state machine.
 *
 * @param input Mask configuration to be written.
 * @param mask_index Index of the mask to which the configuration will be written.
 * @param sm_number State machine number containing the mask to which the configuration will be written.
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_mask_t input, lis3dshtr_mask_selection_t mask_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (mask_index == LIS3DSHTR_MASK_A)
        address = MASK1_A;
    else if (mask_index == LIS3DSHTR_MASK_B)
        address = MASK1_B;

    if (address != 0) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
            address += MASK2_A - MASK1_A;
        }
        success = write((uint8_t *)&input, (lis3dshtr_register_t)address);
    }

    return success;
}

/**
 * Write settings to a state machine (SETTm).
 *
 * State machine settings are the features enabled or disabled in the machines runtime.
 * These settings differ from the "configuration", which mostly dictate whether or not the machine is active.
 * Not a great distinction, but not mine. Blame STM's datasheet.
 *
 * @param input Settings to be written.
 * @param sm_number State machine to which settings will be written (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if the write was successful.
 */
bool LIS3DSHTR::write(lis3dshtr_state_machine_setting_t input, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = write((uint8_t *)&input, SETT1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = write((uint8_t *)&input, SETT2);
    return success;
}

////////////////////////////////////////////////////////////////////////////////
// Read methods

/**
 * Read the raw temperature from the device (OUT_T).
 *
 * This method reads the value, which is a diff from 25°C.
 * The temperature is given in °C to the nearest integer.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_raw_temp_t &output) { return read((uint8_t *)&output, OUT_T); }

/**
 * Read the temperature from the device (OUT_T).
 *
 * The offset is applied to the raw reading.
 * The temperature is given in °C to the nearest integer.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_temp_t &output) {
    lis3dshtr_raw_temp_t temp;
    bool success = read(temp);
    output = temp.raw + LIS3DSHTR_TEMPERATURE_OFFSET;
    return success;
}

/**
 * Read the info registers (INFO1, INFO2)
 *
 * Not entirely sure why you'd need to read from either register, but hey.
 *
 * @param output Container in which read data is stored.
 * @param register_number Index of the register from which data is read ([1,2]).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_info_t &output, uint8_t register_number) {
    bool success = false;
    if (register_number == 1)
        success = read((uint8_t *)&output, INFO1);
    else if (register_number == 2)
        success = read((uint8_t *)&output, INFO2);
    return success;
}

/**
 * Read from the device ID register (WHO_AM_I).
 *
 * The device ID is fixed at 0x3F.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_who_am_i_t &output) { return read((uint8_t *)&output, WHO_AM_I); }

/**
 * Read an offset correction from the device (OFF_x)
 *
 * The read will fail if an invalid axis is supplied.
 * Offsets are read on a per-axis basis only.
 *
 * @param output Container in which read data is stored.
 * @param axis Axis from which the correction is read (LIS3DSHTR_[X,Y,Z]_AXIS).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_offset_t &output, lis3dshtr_axis_selection_t axis) {
    bool success = false;
    if (axis == LIS3DSHTR_X_AXIS)
        success = read((uint8_t *)&output, OFF_X);
    else if (axis == LIS3DSHTR_Y_AXIS)
        success = read((uint8_t *)&output, OFF_Y);
    else if (axis == LIS3DSHTR_Z_AXIS)
        success = read((uint8_t *)&output, OFF_Z);
    return success;
}

/**
 * Read a constant shift from the device (CS_x)
 *
 * The read will fail if an invalid axis is supplied.
 * Constant shifts are read on a per-axis basis only.
 *
 * @param output Container in which read data is stored.
 * @param axis Axis from which the constant shift is read (LIS3DSHTR_[X,Y,Z]_AXIS).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_constant_shift_t &output, lis3dshtr_axis_selection_t axis) {
    bool success = false;
    if (axis == LIS3DSHTR_X_AXIS)
        success = read((uint8_t *)&output, CS_X);
    else if (axis == LIS3DSHTR_Y_AXIS)
        success = read((uint8_t *)&output, CS_Y);
    else if (axis == LIS3DSHTR_Z_AXIS)
        success = read((uint8_t *)&output, CS_Z);
    return success;
}

/**
 * Read the shared long counter value (LC).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_lc_t &output) { return read((uint8_t *)&output, LC_L, 2); }

/**
 * Read from the device state register (STAT).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_stat_t &output) { return read((uint8_t *)&output, STAT); }

/**
 * Read the peak value detected by a state machine (PEAK1, PEAK2).
 *
 * Peak detection must be enabled in SETTm for useful data to appear in these registers.
 * @see lis3dshtr_state_machine_setting_t
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_peak_t &output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)&output, PEAK1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)&output, PEAK2);
    return success;
}

/**
 * Read a coefficient from the vector filter (VFC_n).
 *
 * The vector filter is a 7th-order anti-symmetric FIR filter with the following transfer function:
 * Xv_filt = (x0- x7) coeff0 + (x1-x6) coeff1+ (x2-x5) coeff2 + (x3-x4) coeff3
 * The vector filter must be enabled to be used.
 * The read will fail if a valid coefficient index is not supplied.
 *
 * @see lis3dshtr_interrupt_config_t
 *
 * @param output Container in which read data is stored.
 * @param coefficient_index Index of the coefficient to be read (LIS3DSHTR_VFC_[1-4] or LIS3DSHTR_VECTOR_COEFF_[0-3]).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_vector_filter_coeff_t &output, lis3dshtr_vector_filter_coefficient_selection_t coefficient_index) {
    bool success = false;
    if (coefficient_index == LIS3DSHTR_VFC_1)
        success = read((uint8_t *)&output, VFC_1);
    else if (coefficient_index == LIS3DSHTR_VFC_2)
        success = read((uint8_t *)&output, VFC_2);
    else if (coefficient_index == LIS3DSHTR_VFC_3)
        success = read((uint8_t *)&output, VFC_3);
    else if (coefficient_index == LIS3DSHTR_VFC_4)
        success = read((uint8_t *)&output, VFC_4);

    return success;
}

/**
 * Read the device's power and sampling configuration (CTRL_REG4).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_power_config_t &output) { return read((uint8_t *)&output, CTRL_REG4); }

/**
 * Read a configuration from one of the device's state machines (CTRL_REG1, CTRL_REG2).
 *
 * This configuration register is mostly used to turn the state machine on and off, and interrupt routing.
 * State machine features are handled in the SETTm registers.
 * @see lis3dshtr_state_machine_setting_t
 *
 * @param output Container in which read data is stored.
 * @param sm_number State machine from which the configuration is read (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_state_machine_config_t &output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)&output, CTRL_REG1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)&output, CTRL_REG2);
    return success;
}

/**
 * Read the device's interrupt configuration (CTRL_REG3).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_interrupt_config_t &output) { return read((uint8_t *)&output, CTRL_REG3); }

/**
 * Read the scale configuration from the device (CTRL_REG5)
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_scale_config_t &output) { return read((uint8_t *)&output, CTRL_REG5); }

/**
 * Read the device's FIFO configuration (CTRL_REG6).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_fifo_config_t &output) { return read((uint8_t *)&output, CTRL_REG6); }

/**
 * Read the device's status register (STATUS).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_status_t &output) { return read((uint8_t *)&output, STATUS); }

/**
 * Read the latest acceleration measurement from the device (X_L -> Z_H).
 *
 * All 3 axes are read at once.
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_raw_data_t &output) { return read((uint8_t *)&output, X_L, LIS3DSHTR_ENTRY_SIZE); }

/**
 * Read the roll and pitch from the device (X_L -> Z_H).
 *
 * Roll and pitch are derived terms from the static acceleration of the device.
 * Calculating the pitch and roll while the device is in motion will result in an inaccurate result.
 * Pitch and roll are measured relative to the direction of net acceleration, which is usually gravity.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_roll_pitch_data_t &output) {
    bool success;
    lis3dshtr_raw_data_t raw_data;
    success = read(raw_data);

    // Calculate z-sign for roll correction
    int8_t z_sign;
    if (raw_data.z >= 0)
        z_sign = 1;
    else
        z_sign = -1;

    // See Freescale Application Note AN3461 - Tilt Sensing Using a Three-Axis Accelerometer
    output.pitch = atan2(-raw_data.x, sqrt(raw_data.y * raw_data.y + raw_data.z * raw_data.z));                  // Eq. 26
    output.roll = atan2(raw_data.y, z_sign * sqrt(raw_data.z * raw_data.z + (raw_data.x * raw_data.x * 1E-6)));  // Eq. 38

    return success;
}

/**
 * Read in data from the FIFO buffer (X_L -> Z_H).
 *
 * All available entries are read into the output buffer, but no more than necessary.
 *
 * The FIFO buffer must be enabled for this method to function correctly.
 * @see lis3dshtr_fifo_control_t
 *
 * Automatic address incrementing must also be enabled for this method to function correctly.
 * @see automatic_register_address_incrementing_enabled
 * @see lis3dshtr_fifo_config_t
 *
 * @param output Container in which read data is stored.
 * @return Number of entries read from FIFO buffer.
 */
uint8_t LIS3DSHTR::read(lis3dshtr_fifo_data_t &output) {
    bool success = false;
    lis3dshtr_fifo_status_t fifo_status;
    read(fifo_status);  // Find fifo level

    if (fifo_status.fifo_level > 0) {
        // TODO check that data is read in the correct sequence...
        success = read((uint8_t *)&output, X_L, fifo_status.fifo_level * LIS3DSHTR_ENTRY_SIZE);
    }

    return fifo_status.fifo_level * success;
}

/**
 * Read the configuration from the FIFO control register (FIFO_CTRL).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_fifo_control_t &output) { return read((uint8_t *)&output, FIFO_CTRL); }

/**
 * Read the contents of a state machine's code register (STm_n).
 *
 * The entire code register is read at once.
 *
 * @param output Container in which read data is stored.
 * @param sm_number State machine from which the code register is read (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_state_machine_code_register_t *output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)output, ST1_1, LIS3DSHTR_OP_CODE_REGISTER_SIZE);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)output, ST2_1, LIS3DSHTR_OP_CODE_REGISTER_SIZE);
    return success;
}

/**
 * Read an op code from a state machine's code register (STm_n).
 *
 * @param output Container in which read data is stored.
 * @param sm_number State machine from which the op code will be read.
 * @param position Position in the code register from which the op code will be read (0-15).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_op_code_t &output, uint8_t position, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (position < LIS3DSHTR_OP_CODE_REGISTER_SIZE) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
            success = read((uint8_t *)&output, lis3dshtr_register_t(ST1_1 + position));
        else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
            success = read((uint8_t *)&output, lis3dshtr_register_t(ST2_1 + position));
    }
    return success;
}

/**
 * Read the FIFO status register (FIFO_SRC).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_fifo_status_t &output) { return read((uint8_t *)&output, FIFO_SRC); }

/**
 * Read the value of a short counter (TIM3_m, TIM4_m).
 *
 * @param output Container in which read data is stored.
 * @param counter_index Index of the counter to which the value is written (LIS3DSHTR_TIM[3, 4])
 * @param sm_number State machine from which the configuration is read (LIS3DSHTR_STATE_MACHINE_[1,2]).
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_short_counter_t &output, lis3dshtr_short_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (counter_index == LIS3DSHTR_TIM4)
        address = TIM4_1;
    else if (counter_index == LIS3DSHTR_TIM3)
        address = TIM3_1;

    if (address != 0) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
            address += (TIM4_2 - TIM4_1);
        }
        success = read((uint8_t *)&output, (lis3dshtr_register_t)address);
    }

    return success;
}

/**
 * Read a value from a long counter (LC).
 *
 * This function is only used to read from the shared long counter, as it's the only long counter without a state machine number.
 * It's a redundant method, but its existance allows a consistent reading style from all long counters.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_long_counter_t &output, lis3dshtr_long_counter_selection_t counter_index) {
    bool success = false;
    if (counter_index == LIS3DSHTR_LC) {
        lis3dshtr_lc_t lc;
        success = read(lc);
        output = lc;
    }
    return success;
}

/**
 * Read a value from a long counter (LC, TCm, TIM1_m, TIM2_m).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_long_counter_t &output, lis3dshtr_long_counter_selection_t counter_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (counter_index == LIS3DSHTR_LC) {
        lis3dshtr_lc_t lc;
        success = read(lc);
        output = lc;
    }

    else {
        if (counter_index == LIS3DSHTR_TC)
            address = TC1;
        else if (counter_index == LIS3DSHTR_TIM1)
            address = TIM1_1;
        else if (counter_index == LIS3DSHTR_TIM2)
            address = TIM2_1;

        if (address != 0) {
            if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
                address += (TC2 - TC1);
            }
            success = read((uint8_t *)&output, (lis3dshtr_register_t)address, 2);
        }
    }

    return success;
}

/**
 * Read the value from the shared threshold (THRS3).
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_thrs3_t &output) { return read((uint8_t *)&output, THRS3); }

/**
 * Read a value from a threshold (THRS3).
 *
 * This function is only used to read from the shared threshold (THRS3), as it's the only threshold without a state machine number.
 * It's a redundant method, but its existance allows a consistent reading style to all thresholds.
 *
 * @param output Container in which read data is stored.
 * @param threshold_index Index of the threshold from which the value will be read (LIS3DSHTR_THRS3)
 * @return True if the read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_threshold_t &output, lis3dshtr_threshold_selection_t threshold_index) {
    bool success = false;
    if (threshold_index == LIS3DSHTR_THRS3) {
        lis3dshtr_thrs3_t thres;
        success = read(thres);
        output = thres.raw;
    }
    success = read(output);
    return success;
}

/**
 * Read a value from a threshold (THRS1_m, THRS2_m, THRS3).
 *
 * @param output Container in which read data is stored.
 * @param threshold_index Index of the threshold from which the value will be read (LIS3DSHTR_THRS[1,2,3])
 * @param sm_number State machine from which the threshold is read (LIS3DSHTR_STATE_MACHINE_[1,2]). Does not apply to LIS3DSHTR_THRS3.
 * @return True if the read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_threshold_t &output, lis3dshtr_threshold_selection_t threshold_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (threshold_index == LIS3DSHTR_THRS3) {
        success = read(output, threshold_index);
    } else {
        if (threshold_index == LIS3DSHTR_THRS1) address = THRS1_1;
        if (threshold_index == LIS3DSHTR_THRS2) address = THRS2_1;

        if (address != 0) {
            if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
                address += (THRS1_2 - THRS1_1);
            }
            success = read((uint8_t *)&output, (lis3dshtr_register_t)address);
        }
    }
    return success;
}

/**
 * Read a mask configuration from a state machine (MASKm_A, MASKm_B).
 *
 * @param output Container in which read data is stored.
 * @param mask_index Index of the mask from which the configuration will be read.
 * @param sm_number State machine number containing the mask from which the configuration will be read.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_mask_t &output, lis3dshtr_mask_selection_t mask_index, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    uint8_t address = 0;

    if (mask_index == LIS3DSHTR_MASK_A)
        address = MASK1_A;
    else if (mask_index == LIS3DSHTR_MASK_B)
        address = MASK1_B;

    if (address != 0) {
        if (sm_number == LIS3DSHTR_STATE_MACHINE_2) {
            address += MASK2_A - MASK1_A;
        }
        success = read((uint8_t *)&output, (lis3dshtr_register_t)address);
    }

    return success;
}

/**
 * Read settings from a state machine (SETTm).
 *
 * State machine settings are the features enabled or disabled in the machines runtime.
 * These settings differ from the "configuration", which mostly dictate whether or not the machine is active.
 * Not a great distinction, but not mine. Blame STM's datasheet.
 * @see lis3dshtr_state_machine_config_t
 *
 * @param output Container in which read data is stored.
 * @param sm_number State machine number containing the register to be read.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_state_machine_setting_t &output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)&output, SETT1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)&output, SETT2);
    return success;
}

/**
 * Read the operational pointer from a state machine (PRm).
 *
 * @param output Container in which read data is stored.
 * @param sm_number State machine number containing the register to be read.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_state_machine_pointers_t &output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)&output, PR1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)&output, PR2);
    return success;
}

/**
 * Read the output flags of a state machine (OUTS1, OUTS2)
 * @param output Container in which read data is stored.
 * @param sm_number State machine number containing the register to be read.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_state_machine_output_flags_t &output, lis3dshtr_state_machine_selection_t sm_number) {
    bool success = false;
    if (sm_number == LIS3DSHTR_STATE_MACHINE_1)
        success = read((uint8_t *)&output, OUTS1);
    else if (sm_number == LIS3DSHTR_STATE_MACHINE_2)
        success = read((uint8_t *)&output, OUTS2);
    return success;
}

/**
 * Read the decimation counter of state machine 2.
 *
 * @param output Container in which read data is stored.
 * @return True if read was successful.
 */
bool LIS3DSHTR::read(lis3dshtr_decimation_counter_t &output) { return read((uint8_t *)&output, DES2); }