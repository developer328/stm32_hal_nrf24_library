/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
#include "NRF24.h"

extern SPI_HandleTypeDef hspiX;


void csn_high(void){
	HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin, 1);
}

void csn_low(void){
	HAL_GPIO_WritePin(csn_gpio_port, csn_gpio_pin, 0);
}

void ce_high(void){
	HAL_GPIO_WritePin(ce_gpio_port, ce_gpio_pin, 1);
}

void ce_low(void){
	HAL_GPIO_WritePin(ce_gpio_port, ce_gpio_pin, 0);
}

void nrf24_w_reg(uint8_t reg, uint8_t *data, uint8_t size){

	uint8_t cmd = W_REGISTER | reg;

	csn_low();

	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Transmit(&hspiX, data, size, spi_w_timeout);

	csn_high();
}

uint8_t nrf24_r_reg(uint8_t reg, uint8_t size){
	uint8_t cmd = R_REGISTER | reg;
	uint8_t data = 0;

	csn_low();

	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Receive(&hspiX, &data, size, spi_r_timeout);

	csn_high();

	return data;
}

void nrf24_w_spec_cmd(uint8_t cmd){
	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
}

void nrf24_w_spec_reg(uint8_t *data, uint8_t size){
	HAL_SPI_Transmit(&hspiX, data, size, spi_w_timeout);
}

void nrf24_r_spec_reg(uint8_t *data, uint8_t size){
	HAL_SPI_Receive(&hspiX, data, size, spi_r_timeout);
}

void nrf24_pwr_up(void){
	uint8_t data = 0;

	data = nrf24_r_reg(CONFIG, 1);

	data |= (1 << PWR_UP);

	nrf24_w_reg(CONFIG, &data, 1);
}

void nrf24_pwr_dwn(void){
	uint8_t data = 0;

	data = nrf24_r_reg(CONFIG, 1);

	data &= ~(1 << PWR_UP);

	nrf24_w_reg(CONFIG, &data, 1);
}

void nrf24_tx_pwr(uint8_t pwr){
	uint8_t data = 0;

	data = nrf24_r_reg(RF_SETUP, 1);

	data &= 184;

	data |= (pwr << RF_PWR);

	nrf24_w_reg(RF_SETUP, &data, 1);
}

void nrf24_data_rate(uint8_t bps){
	uint8_t data = 0;

	data = nrf24_r_reg(RF_SETUP, 1);

	data &= ~(1 << RF_DR_LOW) & ~(1 << RF_DR_HIGH);

	if(bps == _2mbps){
		data |= (1 << RF_DR_HIGH);
	}else if(bps == _250kbps){
		data |= (1 << RF_DR_LOW);
	}

	nrf24_w_reg(RF_SETUP, &data, 1);
}

void nrf24_set_channel(uint8_t ch){
	nrf24_w_reg(RF_CH, &ch, 1);
}

void nrf24_open_tx_pipe(uint8_t *addr){
	nrf24_w_reg(TX_ADDR, addr, 5);
}

void nrf24_pipe_pld_size(uint8_t pipe, uint8_t size){
	if(size > 32){
		size = 32;
	}

	switch(pipe){
	case 0:
		nrf24_w_reg(RX_PW_P0, &size, 1);

		break;
	case 1:
		nrf24_w_reg(RX_PW_P1, &size, 1);

		break;
	case 2:
		nrf24_w_reg(RX_PW_P2, &size, 1);

		break;
	case 3:
		nrf24_w_reg(RX_PW_P3, &size, 1);

		break;
	case 4:
		nrf24_w_reg(RX_PW_P4, &size, 1);

		break;
	case 5:
		nrf24_w_reg(RX_PW_P5, &size, 1);

		break;
	}
}

void nrf24_open_rx_pipe(uint8_t pipe, uint8_t *addr){

	uint8_t data = 0;

	data = nrf24_r_reg(EN_RXADDR, 1);

	switch(pipe){
	case 0:
		nrf24_w_reg(RX_ADDR_P0, addr, 5);

		data |= (1 << ERX_P0);
		break;
	case 1:
		nrf24_w_reg(RX_ADDR_P1, addr, 5);

		data |= (1 << ERX_P1);
		break;
	case 2:
		nrf24_w_reg(RX_ADDR_P2, addr, 1);

		data |= (1 << ERX_P2);
		break;
	case 3:
		nrf24_w_reg(RX_ADDR_P3, addr, 1);

		data |= (1 << ERX_P3);
		break;
	case 4:
		nrf24_w_reg(RX_ADDR_P4, addr, 1);

		data |= (1 << ERX_P4);
		break;
	case 5:
		nrf24_w_reg(RX_ADDR_P5, addr, 1);

		data |= (1 << ERX_P5);
		break;
	}

	nrf24_w_reg(EN_RXADDR, &data, 1);
}

void nrf24_cls_rx_pipe(uint8_t pipe){
	uint8_t data = nrf24_r_reg(EN_RXADDR, 1);

	data &= ~(1 << pipe);

	nrf24_w_reg(EN_RXADDR, &data, 1);
}

void nrf24_set_crc(uint8_t en_crc, uint8_t crc0){
	uint8_t data = nrf24_r_reg(CONFIG, 1);

	data &= ~(1 << EN_CRC) & ~(1 << CRCO);

	data |= (en_crc << EN_CRC) | (crc0 << CRCO);

	nrf24_w_reg(CONFIG, &data, 1);
}

void nrf24_set_addr_width(uint8_t bytes){
	bytes -= 2;
	nrf24_w_reg(SETUP_AW, &bytes, 1);
}

void nrf24_flush_tx(void){
	csn_low();
	nrf24_w_spec_cmd(FLUSH_TX);
	csn_high();
}

void nrf24_flush_rx(void){
	csn_low();
	nrf24_w_spec_cmd(FLUSH_RX);
	csn_high();
}

uint8_t nrf24_r_status(void){
	uint8_t data = 0;
	uint8_t cmd = NOP_CMD;

	csn_low();
	HAL_SPI_TransmitReceive(&hspiX, &cmd, &data, 1, spi_rw_timeout);
	csn_high();

	return data;
}

void nrf24_clear_rx_dr(void){
	uint8_t data = 0;

	data = nrf24_r_status();

	data |= (1 << RX_DR);

	nrf24_w_reg(STATUS, &data, 1);
}

void nrf24_clear_tx_ds(void){
	uint8_t data = 0;

	data = nrf24_r_status();

	data |= (1 << TX_DS);

    nrf24_w_reg(STATUS, &data, 1);
}

void nrf24_clear_max_rt(void){
	uint8_t data = 0;

	data = nrf24_r_status();

	data |= (1 << MAX_RT);

    nrf24_w_reg(STATUS, &data, 1);
}

uint8_t nrf24_read_bit(uint8_t reg, uint8_t bit){

	if(nrf24_r_reg(reg, 1) & (1 << bit)){
		return 1;
	}

	return 0;
}

void nrf24_set_bit(uint8_t reg, uint8_t bit, uint8_t val){
	uint8_t data = 0;

	data = nrf24_r_reg(reg, 1);

	if(val){
		data |= (1 << bit);
	}else{
		data &= ~(1 << bit);
	}

    nrf24_w_reg(reg, &data, 1);
}

uint8_t nrf24_r_pld_wid(void){
	uint8_t width = 0;

	csn_low();
	nrf24_w_spec_cmd(R_RX_PL_WID);
	nrf24_r_spec_reg(&width, 1);
	csn_high();

	return width;
}

void nrf24_listen(void){
	uint8_t data = 0;

	data = nrf24_r_reg(CONFIG, 1);

	data |= (1 << PRIM_RX);

	nrf24_w_reg(CONFIG, &data, 1);

	ce_high();
}

void nrf24_stop_listen(void){
	uint8_t data = 0;

	data = nrf24_r_reg(CONFIG, 1);

	data &= ~(1 << PRIM_RX);

	nrf24_w_reg(CONFIG, &data, 1);
}

void nrf24_dpl(uint8_t en){
	uint8_t feature = nrf24_r_reg(FEATURE, 1);

	if(en == enable){
		feature |= (1 << EN_DPL);
	}else{
		feature &= ~(1 << EN_DPL);
	}

	nrf24_w_reg(FEATURE, &feature, 1);
}

void nrf24_set_rx_dpl(uint8_t pipe, uint8_t en){

	uint8_t dynpd = nrf24_r_reg(DYNPD, 1);

	if(pipe > 5){
		pipe = 5;
	}

	if(en){
		dynpd |= (1 << pipe);
	}else{
		dynpd &= ~(1 << pipe);
	}

	nrf24_w_reg(DYNPD, &dynpd, 1);
}

void nrf24_auto_ack(uint8_t pipe, uint8_t ack){

	if(pipe > 5){
		pipe = 5;
	}

	uint8_t enaa = nrf24_r_reg(EN_AA, 1);

	if(ack){
		enaa |= (1 << pipe);
	}else{
		enaa &= ~(1 << pipe);
	}

	nrf24_w_reg(EN_AA, &enaa, 1);
}

void nrf24_auto_ack_all(uint8_t ack){
	uint8_t enaa = nrf24_r_reg(EN_AA, 1);

	if(ack){
		enaa = 63;
	}else{
		enaa = 0;
	}

	nrf24_w_reg(EN_AA, &enaa, 1);
}

void nrf24_en_ack_pld(uint8_t en){
	uint8_t feature = nrf24_r_reg(FEATURE, 1);

	if(en){
		feature |= (1 << EN_ACK_PAY);
	}else{
		feature &= ~(1 << EN_ACK_PAY);
	}

	nrf24_w_reg(FEATURE, &feature, 1);
}

void nrf24_en_dyn_ack(uint8_t en){
	uint8_t feature = nrf24_r_reg(FEATURE, 1);

	if(en){
		feature |= (1 << EN_DYN_ACK);
	}else{
		feature &= ~(1 << EN_DYN_ACK);
	}

	nrf24_w_reg(FEATURE, &feature, 1);
}

void nrf24_auto_retr_delay(uint8_t delay){
	uint8_t data = nrf24_r_reg(SETUP_RETR, 1);

	data &= 15;

	data |= (delay << ARD);

	nrf24_w_reg(SETUP_RETR, &data, 1);
}

void nrf24_auto_retr_limit(uint8_t limit){
	uint8_t data = nrf24_r_reg(SETUP_RETR, 1);

	data &= 240;

	data |= (limit << ARC);

	nrf24_w_reg(SETUP_RETR, &data, 1);
}

void nrf24_type_to_uint8_t(size_t in, uint8_t* out, uint16_t size){
	for(uint16_t i = 0; i < size; i++){
		out[i] = (((in & (255 << (i*8)))) >> (i*8));
	}
}

size_t nrf24_uint8_t_to_type(uint8_t* in, uint16_t size){
	size_t out = 0;

	for(uint16_t i = 0; i < size; i++){
		out |= (in[i] << (8*i));
	}

	return out;
}


uint8_t nrf24_transmit(uint8_t *data, uint8_t size){

	ce_low();

	uint8_t cmd = W_TX_PAYLOAD;

	csn_low();
	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Transmit(&hspiX, data, size, spi_w_timeout);
	csn_high();

	ce_high();
	HAL_Delay(1);
	ce_low();

	if(nrf24_r_status() & (1 << MAX_RT)){
		nrf24_clear_max_rt();
		nrf24_flush_tx();
		return 1;
	}

	return 0;
}

void nrf24_transmit_no_ack(uint8_t *data, uint8_t size){

	ce_low();

	uint8_t cmd = W_TX_PAYLOAD_NOACK;

	csn_low();
	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Transmit(&hspiX, data, size, spi_w_timeout);
	csn_high();

	ce_high();
	HAL_Delay(1);
	ce_low();
}

void nrf24_transmit_rx_ack_pld(uint8_t pipe, uint8_t *data, uint8_t size){

	if(pipe > 5){
		pipe = 5;
	}

	uint8_t cmd = (W_ACK_PAYLOAD | pipe);

	csn_low();
	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Transmit(&hspiX, data, size, spi_w_timeout);
	csn_high();

}

uint8_t nrf24_carrier_detect(void){
	return nrf24_r_reg(RPD, 1);
}

uint8_t nrf24_data_available(void){

 	uint8_t reg_dt = nrf24_r_reg(FIFO_STATUS, 1);

	if(!(reg_dt & (1 << RX_EMPTY))){
		return 1;
	}

	return 0;
}

void nrf24_receive(uint8_t *data, uint8_t size){
	uint8_t cmd = R_RX_PAYLOAD;

	csn_low();
	HAL_SPI_Transmit(&hspiX, &cmd, 1, spi_w_timeout);
	HAL_SPI_Receive(&hspiX, data, size, spi_r_timeout);
	csn_high();

	nrf24_clear_rx_dr();
}

void nrf24_defaults(void){
	ce_low();

	nrf24_pwr_dwn();
	nrf24_tx_pwr(3);
	nrf24_data_rate(_1mbps);
	nrf24_set_channel(2);
	nrf24_set_crc(no_crc, _1byte);
	nrf24_set_addr_width(5);
	nrf24_flush_tx();
	nrf24_flush_rx();
	nrf24_clear_rx_dr();
	nrf24_clear_tx_ds();
	nrf24_clear_max_rt();
	nrf24_stop_listen();
	nrf24_dpl(disable);
	nrf24_en_ack_pld(disable);
	nrf24_en_dyn_ack(disable);
	nrf24_auto_retr_delay(0);
	nrf24_auto_retr_limit(3);


	for(uint8_t i = 0; i < 5; i++){
		nrf24_pipe_pld_size(i, 0);
		nrf24_cls_rx_pipe(i);
		nrf24_set_rx_dpl(i, disable);
		nrf24_auto_ack(i, enable);
	}

	ce_high();
}

void nrf24_init(void){

	nrf24_pwr_up();

	nrf24_flush_tx();
	nrf24_flush_rx();

	nrf24_clear_rx_dr();
	nrf24_clear_tx_ds();
	nrf24_clear_max_rt();
}

