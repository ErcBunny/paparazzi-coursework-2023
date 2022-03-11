/*
 * Copyright (C) 2022 OpenUAS
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ca_am7.c"
 * @author OpenUAS
 */

#include "modules/sensors/ca_am7.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"

static abi_event AM7_out;
uint8_t sending_msg_id;
struct am7_data_in myam7_data_in;
struct am7_data_out myam7_data_out;
float extra_data_in[255], extra_data_out[255];
uint16_t buffer_in_counter = 0;
uint32_t missed_packets = 0;
uint16_t ca7_message_frequency_RX = 0;
uint32_t received_packets = 0;
float last_ts = 0;
static uint8_t am7_msg_buf_in[sizeof(struct am7_data_in)*2]  __attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void am7_downlink(struct transport_tx *trans, struct link_device *dev)
{
	   pprz_msg_send_AM7_IN(trans, dev, AC_ID, &myam7_data_in.motor_1_cmd_int, &myam7_data_in.motor_2_cmd_int, &myam7_data_in.motor_3_cmd_int,
		 	  &myam7_data_in.motor_4_cmd_int, &myam7_data_in.el_1_cmd_int, &myam7_data_in.el_2_cmd_int, &myam7_data_in.el_3_cmd_int,
		 	  &myam7_data_in.el_4_cmd_int, &myam7_data_in.az_1_cmd_int,  &myam7_data_in.az_2_cmd_int, &myam7_data_in.az_3_cmd_int,
              &myam7_data_in.az_4_cmd_int, &myam7_data_in.theta_cmd_int, &myam7_data_in.phi_cmd_int,&myam7_data_in.n_iteration,
              &myam7_data_in.n_evaluation, &myam7_data_in.elapsed_time_us, &myam7_data_in.residual_ax_int,&myam7_data_in.residual_ay_int, &myam7_data_in.residual_az_int,
              &myam7_data_in.residual_p_dot_int, &myam7_data_in.residual_q_dot_int,&myam7_data_in.residual_r_dot_int, &missed_packets, &ca7_message_frequency_RX,
              &myam7_data_in.rolling_msg_in, &myam7_data_in.rolling_msg_in_id);
}
static void am7_uplink(struct transport_tx *trans, struct link_device *dev)
{
	   pprz_msg_send_AM7_OUT(trans, dev, AC_ID, &myam7_data_out.motor_1_state_int, &myam7_data_out.motor_2_state_int, &myam7_data_out.motor_3_state_int,
		 	  &myam7_data_out.motor_4_state_int, &myam7_data_out.el_1_state_int, &myam7_data_out.el_2_state_int, &myam7_data_out.el_3_state_int,
		 	  &myam7_data_out.el_4_state_int, &myam7_data_out.az_1_state_int,  &myam7_data_out.az_2_state_int, &myam7_data_out.az_3_state_int,
              &myam7_data_out.az_4_state_int, &myam7_data_out.theta_state_int, &myam7_data_out.phi_state_int, &myam7_data_out.psi_state_int,
              &myam7_data_out.gamma_state_int, &myam7_data_out.p_state_int, &myam7_data_out.q_state_int, &myam7_data_out.r_state_int,
              &myam7_data_out.airspeed_state_int, &myam7_data_out.pseudo_control_ax_int, &myam7_data_out.pseudo_control_ay_int, &myam7_data_out.pseudo_control_az_int,
              &myam7_data_out.pseudo_control_p_dot_int,&myam7_data_out.pseudo_control_q_dot_int, &myam7_data_out.pseudo_control_r_dot_int,
              &myam7_data_out.rolling_msg_out, &myam7_data_out.rolling_msg_out_id);

}
#endif

static void data_AM7_out(uint8_t sender_id __attribute__((unused)), float * myam7_data_out_ptr, float * extra_data_out_ptr){

    memcpy(&myam7_data_out,myam7_data_out_ptr,sizeof(struct am7_data_out));
    memcpy(&extra_data_out,extra_data_out_ptr, sizeof(extra_data_out) );

    //Increase the counter to track the sending messages:
    myam7_data_out.rolling_msg_out = extra_data_out[sending_msg_id];
    myam7_data_out.rolling_msg_out_id = sending_msg_id;
    sending_msg_id++;
    if(sending_msg_id == 255){
        sending_msg_id = 0;
    }

    //Send the message over serial to the Raspberry pi:
    uint8_t *buf_send = (uint8_t *)&myam7_data_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct am7_data_out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    myam7_data_out.checksum_out = checksum_out_local;
    //Send bytes
    uart_put_byte(&(AM7_PORT), 0, START_BYTE);
    for(uint8_t i = 0; i < sizeof(struct am7_data_out) ; i++){
        uart_put_byte(&(AM7_PORT), 0, buf_send[i]);
    }
}

void am7_init() 
{
    buffer_in_counter = 0;
    sending_msg_id = 0;

    //Init abi bind msg:
    AbiBindMsgAM7_DATA_OUT(ABI_BROADCAST, &AM7_out, data_AM7_out);

 #if PERIODIC_TELEMETRY
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_IN, am7_downlink);
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_OUT, am7_uplink);
 #endif
}

/* Parse the InterMCU message */
void am7_parse_msg_in(void)
{
    memcpy(&myam7_data_in, &am7_msg_buf_in[1], sizeof(struct am7_data_in));
    //Assign the rolling message:
    extra_data_in[myam7_data_in.rolling_msg_in_id] = myam7_data_in.rolling_msg_in;
    //Send msg through ABI:
    AbiSendMsgAM7_DATA_IN(ABI_AM7_DATA_IN_ID, &myam7_data_in, &extra_data_in[0]);
}

void assign_variables(void){

//    myam7_data_out.motor_1_state_int =  1000;
//    myam7_data_out.motor_2_state_int =  1000;
//    myam7_data_out.motor_3_state_int =  1000;
//    myam7_data_out.motor_4_state_int = 1000;
//
//    myam7_data_out.el_1_state_int = 0;
//    myam7_data_out.el_2_state_int = 0;
//    myam7_data_out.el_3_state_int = 0;
//    myam7_data_out.el_4_state_int = 0;
//
//    myam7_data_out.az_1_state_int = 0;
//    myam7_data_out.az_2_state_int = 0;
//    myam7_data_out.az_3_state_int = 0;
//    myam7_data_out.az_4_state_int = 0;
//
//    myam7_data_out.theta_state_int = 0;
//    myam7_data_out.phi_state_int = 0;
//    myam7_data_out.psi_state_int = 0;
//    myam7_data_out.gamma_state_int = 0;
//    myam7_data_out.p_state_int = 0;
//    myam7_data_out.q_state_int = 0;
//    myam7_data_out.r_state_int = 0;
//    myam7_data_out.airspeed_state_int = 0;
//    myam7_data_out.pseudo_control_ax_int = 5670;
//    myam7_data_out.pseudo_control_ay_int = 1046;
//    myam7_data_out.pseudo_control_az_int = 1675;
//    myam7_data_out.pseudo_control_p_dot_int = 1073;
//    myam7_data_out.pseudo_control_q_dot_int = 1034;
//    myam7_data_out.pseudo_control_r_dot_int = 11342;
//    myam7_data_out.rolling_msg_out = 2.54;

}

void am7_periodic(){

    assign_variables();

//    uint8_t *buf_send = (uint8_t *)&myam7_data_out;
//    //Calculating the checksum
//    uint8_t checksum_out_local = 0;
//    for(uint16_t i = 0; i < sizeof(struct am7_data_out) - 1; i++){
//        checksum_out_local += buf_send[i];
//    }
//    myam7_data_out.checksum_out = checksum_out_local;
//    //Send bytes
//    uart_put_byte(&(AM7_PORT), 0, START_BYTE);
//    for(uint8_t i = 0; i < sizeof(struct am7_data_out) ; i++){
//        uart_put_byte(&(AM7_PORT), 0, buf_send[i]);
//    }


}
/* We need to wait for incoming messages */
void am7_event()
{
    if(fabs(get_sys_time_float() - last_ts) > 5){
        received_packets = 0;
        last_ts = get_sys_time_float();
    }
    while(uart_char_available(&(AM7_PORT)) > 0) {
        uint8_t am7_byte_in;
        am7_byte_in = uart_getch(&(AM7_PORT));
        if ((am7_byte_in == START_BYTE) || (buffer_in_counter > 0)) {
            am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
            buffer_in_counter++;
        }
        if (buffer_in_counter > sizeof(struct am7_data_in) ) {
            buffer_in_counter = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct am7_data_in) ; i++){
                checksum_in_local += am7_msg_buf_in[i];
            }
            if(checksum_in_local == am7_msg_buf_in[sizeof(struct am7_data_in)]){
                am7_parse_msg_in();
                received_packets++;
            }
            else {
                missed_packets++;
            }
        }
    }
    ca7_message_frequency_RX = (uint16_t) received_packets/(get_sys_time_float() - last_ts);
}

