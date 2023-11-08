/*
 * serial.c
 *
 *  Created on: 2023. 11. 3.
 *      Author: JH
 */

#include "board.h"
#include "version.h"
#include "alarm.h"
#include "parameter.h"
#include "control.h"
#include "io_process.h"
#include "serial.h"

uint16_t rx_buf[5];
uint16_t tx_buf[30];



uint16_t decode_rx_data(void);
void make_tx_data_frame_32bit(uint16_t *index, uint16_t *txdata, uint32_t req_data);
void make_tx_data_frame_16bit(uint16_t *index, uint16_t *txdata, uint16_t req_data);
void make_tx_data_frame_8bit(uint16_t *index, uint16_t *txdata, uint16_t req_data);


void process_serial_communication(void)
{
    uint16_t tx_lenth = 0;
    uint16_t i = 0;

    if(check_rx_buf() != 0)
    { tx_lenth = decode_rx_data(); }

    if(tx_lenth != 0)
    {
        if(tx_buf[INDEX_CMD] == ERROR_INVALID_CMD)
        { write_tx_data(tx_buf[INDEX_CMD]); }
        else
        {
            for(i = 0; i < tx_lenth; i++)
            { write_tx_data(tx_buf[i]); }
        }
    }
}


uint16_t decode_rx_data(void)
{
    uint16_t    index;
    uint32_t    type_as_uint32;
    float32_t   temp_float32;

    read_rx_data(rx_buf);
    tx_buf[INDEX_CMD] = rx_buf[INDEX_CMD];

    switch(rx_buf[INDEX_CMD])
    {
        case CMD_VERSION_INFO:
            make_tx_data_frame_32bit(&index, tx_buf, os_version.Full);
            break;

        case CMD_PRODUCT_INFO:
            type_as_uint32 = PRODUCT_NO;
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            break;

        case CMD_PARAMETER_INFO:
            make_tx_data_frame_8bit(&index, tx_buf, initial_parameter.selftest);
            make_tx_data_frame_8bit(&index, tx_buf, initial_parameter.input_mode);
            make_tx_data_frame_8bit(&index, tx_buf, initial_parameter.motor_direction);
            make_tx_data_frame_8bit(&index, tx_buf, initial_parameter.gain_sel);

            memcpy(&type_as_uint32, &initial_parameter.p_gain, sizeof(initial_parameter.p_gain));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &initial_parameter.d_gain, sizeof(initial_parameter.d_gain));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &initial_parameter.run_current, sizeof(initial_parameter.run_current));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);

            temp_float32 = initial_parameter.run_current * ((float32_t)initial_parameter.stop_current * 0.01);
            memcpy(&type_as_uint32, &temp_float32, sizeof(temp_float32));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);

            make_tx_data_frame_32bit(&index, tx_buf, initial_parameter.resolution);
            break;

        case CMD_ERROR_STATUS:
            make_tx_data_frame_32bit(&index, tx_buf, err_status.Full);
            break;

        case CMD_MONITORING_DATA0:
            memcpy(&type_as_uint32, &motor_detect.motor_ls, sizeof(motor_detect.motor_ls));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &motor_detect.motor_dcr, sizeof(motor_detect.motor_dcr));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            break;

        case CMD_MONITORING_DATA1:
            memcpy(&type_as_uint32, &cmd_pos.total, sizeof(cmd_pos.total));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &cstep.m_cmd_speed, sizeof(cstep.m_cmd_speed));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            make_tx_data_frame_32bit(&index, tx_buf, 0); //actual pulse
            make_tx_data_frame_32bit(&index, tx_buf, 0); //actual rpm
            break;

        case CMD_MONITORING_DATA2:
            memcpy(&type_as_uint32, &cur_pid.ab_real, sizeof(cur_pid.ab_real));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &system_condition.dc_link, sizeof(system_condition.dc_link));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &system_condition.device_temp, sizeof(system_condition.device_temp));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            memcpy(&type_as_uint32, &system_condition.dsp_temp, sizeof(system_condition.dsp_temp));
            make_tx_data_frame_32bit(&index, tx_buf, type_as_uint32);
            break;

        case CMD_MONITORING_DATA3:
            make_tx_data_frame_16bit(&index, tx_buf, driver_status.Full);
            make_tx_data_frame_16bit(&index, tx_buf, input_status.Full);
            make_tx_data_frame_16bit(&index, tx_buf, output_status.Full);
            break;

        default :
            tx_buf[INDEX_CMD] = ERROR_INVALID_CMD;
            break;
    }
    return index;
}

void make_tx_data_frame_32bit(uint16_t *index, uint16_t *txdata, uint32_t req_data)
{
    *index = *index + 1;    txdata[*index] = (uint16_t)(req_data & 0xFF);
    *index = *index + 1;    txdata[*index] = (uint16_t)((req_data >> 8)  & 0xFF);
    *index = *index + 1;    txdata[*index] = (uint16_t)((req_data >> 16) & 0xFF);
    *index = *index + 1;    txdata[*index] = (uint16_t)((req_data >> 24) & 0xFF);
}

void make_tx_data_frame_16bit(uint16_t *index, uint16_t *txdata, uint16_t req_data)
{
    *index = *index + 1;    txdata[*index] = (uint16_t)(req_data & 0xFF);
    *index = *index + 1;    txdata[*index] = (uint16_t)((req_data >> 8)  & 0xFF);
}

void make_tx_data_frame_8bit(uint16_t *index, uint16_t *txdata, uint16_t req_data)
{
    *index = *index + 1;    txdata[*index] = (uint16_t)(req_data & 0xFF);
}


