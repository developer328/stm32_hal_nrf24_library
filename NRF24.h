/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 *
 */


#ifndef NRF_24_H
#define NRF_24_H

enum data_rate {
	_1mbps   = 0,
	_2mbps   = 1,
	_250kbps = 2
};

enum tx_power {
	n18dbm = 0,
	n12dbm = 1,
	n6dbm  = 2,
	_0dbm  = 3
};

enum acknowledgement {
	auto_ack = 1,
	no_auto_ack = 0
};

enum crc_en {
	no_crc = 0,
	en_crc = 1
};

enum crc_rn_sch {
	_1byte = 0,
	_2byte = 1
};

enum {
	enable = 1,
	disable = 0
};



/*
 * Set defaults
 *
 * NOTICE: Don't use it at start of program because it may broke down process
 * 		   after reseting the MCU. This function is for only special situations or testing.
 */
void nrf24_defaults(void);


/*
 * This function sets everything which is necessary for correct start of program
 *
 * NOTICE: Without this function in start of program may cause problems after reseting MCU.
 * 		   MCU reset does not resets NRF24 module
 */
void nrf24_init(void);


//These functions are for controll CE and CSN pins which are selected in NRF24_conf.h
void csn_high(void);
void csn_low(void);
void ce_high(void);
void ce_low(void);


//For read or write nrf24 registers via spi bus
void nrf24_w_reg(uint8_t reg, uint8_t *data, uint8_t size);
uint8_t nrf24_r_reg(uint8_t reg, uint8_t size);


//For write some special commands such as  W_TX_PAYLOAD, FLUSH_RX, ETC
void nrf24_w_spec_cmd(uint8_t cmd);


/*  One of these functions are used after "nrf24_w_spec_cmd" for execute
 * command which means reading or writing relevant Buffer. this functions
 * are not necessary after all special commands such as FLUSH_TX or FLUSH_RX
 */
void nrf24_w_spec_reg(uint8_t *data, uint8_t size);
void nrf24_r_spec_reg(uint8_t *data, uint8_t size);


//Power up nrf24 module
void nrf24_pwr_up(void);


//Power down nrf24 module
void nrf24_pwr_dwn(void);


/* Control nrf24 TX power:
 *
 *  n18dbm = 0 -> Minimum power
 *
 *  n12dbm = 1
 *
 *  n6dbm  = 2
 *
 * _0dbm  = 3 -> Maximum power
 *
 */
void nrf24_tx_pwr(uint8_t pwr);


/* Set NRF24 data rate:
 *
 * _1mbps   = 0
 *
 * _2mbps   = 1
 *
 * _250kbps = 2
 *
 */
void nrf24_data_rate(uint8_t bps);


/*Set channel from 0 to 125 (In summary it has 126 channel from 1400Mhz to 1525Mhz)
 *
 * Frequency -> channel
 *
 * 1400Mhz -> 0
 * 1401Mhz -> 1
 *  ....
 * 1525Mhz -> 125
 */
void nrf24_set_channel(uint8_t ch);


/*Write here pipe address, for example:
 * uint8_t addr[5] = { 0x53, 0x13, 0x01, 0x75, 0x82 };
 */
void nrf24_open_tx_pipe(uint8_t *addr);


/*
 *  Control payload size for each pipe individually (from 0 to 32)
 */
void nrf24_pipe_pld_size(uint8_t pipe, uint8_t size);


/*
 * Open and set rx pipe address (Rx pipes are from 0 to 5)
 *
 * NOTICE: if you are using acknowledgement feature it is obligatory
 *         to set same address to TX and RX 0 pipe.
 */
void nrf24_open_rx_pipe(uint8_t pipe, uint8_t *addr);


/*
 * Close opened RX pipe
 */
void nrf24_cls_rx_pipe(uint8_t pipe);


/*
 * Set CRC:
 *
 * First argument:
 * no_crc = 0 -> Disable CRC
 * en_crc = 1 -> Enable CRC
 *
 * Second argument:
 * 1byte = 0
 *_2byte = 1
 */
void nrf24_set_crc(uint8_t en_crc, uint8_t crc0);


/*
 * Set address width which as default is 5 bytes and it mostly doesn't
 * requires change but if you need to set it for example 3 bytes you
 * won't be able to use this: uint8_t addr[5] = { 0x53, 0x13, 0x01, 0x75, 0x82 };
 * you need to shorten it for instance: uint8_t addr[3] = { 0x53, 0x13, 0x01 };
 */
void nrf24_set_addr_width(uint8_t bytes);


//This function is for clear TX FIFO fully
void nrf24_flush_tx(void);


//This function is for clear RX FIFO fully
void nrf24_flush_rx(void);


/*
 * Read Status register correctly without interfering in processes
 */
uint8_t nrf24_r_status(void);


/*
 * This function is for clear RX_DR bit in NRF24 STATUS register which sets
 * after receiving data in RX FIFO and it must be cleared by writing 1 which
 * sets it to 0
 */
void nrf24_clear_rx_dr(void);


/*
 * This function is for clear TX_DS bit in NRF24 STATUS register which sets
 * after Transmitting data or receiving acknowledgement and it must be cleared
 * by writing 1 which sets it to 0
 */
void nrf24_clear_tx_ds(void);


/*
 * This function is for clear MAX_RT bit in NRF24 STATUS register which sets
 * after auto retransmissions will exceed limit and it must be cleared by writing 1 which
 * sets it to 0
 */
void nrf24_clear_max_rt(void);


/*
 * Read individual bit in preferred register
 */
uint8_t nrf24_read_bit(uint8_t reg, uint8_t bit);


/*
 * Set individual bit in preferred register
 */
void nrf24_set_bit(uint8_t reg, uint8_t bit, uint8_t val);


/*
 * Read received payload size which is also useful for prevent data which size
 * is more than 32 bytes by flushing it "nrf24_flush_rx()"
 */
uint8_t nrf24_r_pld_wid(void);


/*
 * This function turns nrf24 to RX mode also it sets CE pin HIGH
 */
void nrf24_listen(void);


/*
 * This function is for turn off RX mode and go in TX mode
 */
void nrf24_stop_listen(void);


/*
 * Enable or disable Dynamic PayLoad by writing 'enable' or 'disable' as an argument
 */
void nrf24_dpl(uint8_t en);


/*
 * Set individual pipe for dynamic payload.
 * This function must be used after "nrf24_dpl(uint8_t en)" function
 */
void nrf24_set_rx_dpl(uint8_t pipe, uint8_t en);


/*
 * Enable or disable auto acknowledgement for each pipe
 */
void nrf24_auto_ack(uint8_t pipe, uint8_t ack);


/*
 * Enable or disable auto acknowledgement for all pipe together
 */
void nrf24_auto_ack_all(uint8_t ack);


/*
 * Enable or disable acknowledgement with user defined payload
 */
void nrf24_en_ack_pld(uint8_t en);


/*
 * Enable or disable "nrf24_transmit_no_ack"
 */
void nrf24_en_dyn_ack(uint8_t en);


/*
 * Set delay between auto-retransmissions
 */
void nrf24_auto_retr_delay(uint8_t delay);


/*
 * Set auto-retransmissions limit
 */
void nrf24_auto_retr_limit(uint8_t limit);


/*
 * This function is used to convert the not uint8_t varible into uint8_t array which is
 * necessary for transmit correctly and on timing
 */
void nrf24_type_to_uint8_t(size_t in, uint8_t* out, uint16_t size);


/*
 * This function converts received uint8_t array to other one data type for example
 * uint16_t, uint32_t
 */
size_t nrf24_uint8_t_to_type(uint8_t* in, uint16_t size);


/*
 * Transmit data.
 * This function returns 0 if everything is OK but if MAX_RT has been set 1
 * which means target RX device does not responding, it returns 1, this is useful
 * with acknowledgements
 */
uint8_t nrf24_transmit(uint8_t *data, uint8_t size);


/*
 * Transmit in auto_ack mode without request ack packet from RX device
 */
void nrf24_transmit_no_ack(uint8_t *data, uint8_t size);


/*
 * Send ACk with payload packet as a receiver(RX) device
 */
void nrf24_transmit_rx_ack_pld(uint8_t pipe, uint8_t *data, uint8_t size);


/*
 * Detect signal on selected channel above -64dbm
 */
uint8_t nrf24_carrier_detect(void);


/*
 * check if data is in RX FIFO
 */
uint8_t nrf24_data_available(void);


/*
 * Receive data
 */
void nrf24_receive(uint8_t *data, uint8_t size);


#endif

