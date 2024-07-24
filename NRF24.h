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

void csn_high(void);
void csn_low(void);
void ce_high(void);
void ce_low(void);

void nrf24_w_reg(uint8_t reg, uint8_t *data, uint8_t size);
uint8_t nrf24_r_reg(uint8_t reg, uint8_t size);

void nrf24_w_spec_cmd(uint8_t cmd);
void nrf24_w_spec_reg(uint8_t *data, uint8_t size);
void nrf24_r_spec_reg(uint8_t *data, uint8_t size);

void nrf24_pwr_up(void);
void nrf24_pwr_dwn(void);

void nrf24_tx_pwr(uint8_t pwr);
void nrf24_data_rate(uint8_t bps);
void nrf24_set_channel(uint8_t ch);

void nrf24_open_tx_pipe(uint8_t *addr);
void nrf24_pipe_pld_size(uint8_t pipe, uint8_t size);
void nrf24_open_rx_pipe(uint8_t pipe, uint8_t *addr);
void nrf24_cls_rx_pipe(uint8_t pipe);

void nrf24_set_crc(uint8_t en_crc, uint8_t crc0);
void nrf24_set_addr_width(uint8_t bytes);

void nrf24_flush_tx(void);
void nrf24_flush_rx(void);

void nrf24_clear_rx_dr(void);
void nrf24_clear_tx_ds(void);
void nrf24_clear_max_rt(void);

uint8_t nrf24_read_bit(uint8_t reg, uint8_t bit);
void nrf24_set_bit(uint8_t reg, uint8_t bit, uint8_t val);
uint8_t nrf24_r_pld_wid(void);

void nrf24_listen(void);
void nrf24_stop_listen(void);

void nrf24_dpl(uint8_t en);
void nrf24_set_rx_dpl(uint8_t pipe, uint8_t en);

void nrf24_auto_ack(uint8_t pipe, uint8_t ack);
void nrf24_auto_ack_all(uint8_t ack);
void nrf24_en_ack_pld(uint8_t en);
void nrf24_en_dyn_ack(uint8_t en);
void nrf24_auto_retr_delay(uint8_t delay);
void nrf24_auto_retr_limit(uint8_t limit);

uint8_t nrf24_transmit(uint8_t *data, uint8_t size);
void nrf24_transmit_no_ack(uint8_t *data, uint8_t size);
void nrf24_transmit_rx_ack_pld(uint8_t pipe, uint8_t *data, uint8_t size);

uint8_t nrf24_carrier_detect(void);
uint8_t nrf24_data_available(void);
void nrf24_receive(uint8_t *data, uint8_t size);

void nrf24_defaults(void);
void nrf24_init(void);

#endif

