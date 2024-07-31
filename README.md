# **NRF24 STM32 HAL Library**

=====================================================================================


 This library provides functions to interface the NRF24L01+ transceiver module using 
STM32 HAL (Hardware Abstraction Layer). It supports features such as dynamic payload 
length (DPL), automatic acknowledgment (ACK), and various data rates. 

[Tutorial on youtube](https://www.youtube.com/watch?v=a--IXKcEwdQ)

## **Features**

- Dynamic Payload Length (DPL)

- Automatic Acknowledgment (ACK)

- Configurable Data Rates (250 kbps, 1 Mbps, 2 Mbps)

- CRC Configuration (1-byte or 2-byte)

- RF signal power control (n18dbm, n12dbm, n6dbm, _0dbm)

- RF channel selection 

- Pipes control, such as: address changing, closiing/opening pipes

- Simple APIs for Transmitting and Receiving Data


## **Hardware requirements:**

- STM32 microcontroller 

- NRF24L01+ transceiver module

- STlink programmer


## **Software requirements:**

- STM32CubeIDE

- STM32CubeMX 

- STM32CubeProgrammer


## **Hardware Setup (NRF24L01+ module to STM32)**

- VCC to 3.3V
- GND to GND
- CE to GPIO_PIN_x (Configurable)
- CSN to GPIO_PIN_x (Configurable)
- SCK to SPI_SCK_PIN
- MOSI to SPI_MOSI_PIN
- MISO to SPI_MISO_PIN
- IRQ (optional) to GPIO_PIN_x (Configurable)


## **Configurations**

Open NRF24_conf.h and select SPI port, CE and CS GPIO pins and CS and CE pins relevant GPIO ports.


## Getting started**

### **Initialization:**

    csn_high();

    HAL_Delay(5);

    ce_low();

    nrf24_init();



### **Configurations:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(disable);
    nrf24_en_dyn_ack(disable);
    nrf24_dpl(disable);

    nrf24_set_crc(no_crc, _1byte);

    nrf24_tx_pwr(_0dbm);
    nrf24_data_rate(_250kbps);
    nrf24_set_channel(74);
    nrf24_set_addr_width(5);

    nrf24_set_rx_dpl(0, disable);
    nrf24_set_rx_dpl(1, disable);
    nrf24_set_rx_dpl(2, disable);
    nrf24_set_rx_dpl(3, disable);
    nrf24_set_rx_dpl(4, disable);
    nrf24_set_rx_dpl(5, disable);

    nrf24_pipe_pld_size(0, PLD_S);

    nrf24_auto_retr_delay(0);
    nrf24_auto_retr_limit(15);

    uint8_t addr[5] = { 0x53, 0x13, 0x01, 0x75, 0x82 };
    nrf24_open_tx_pipe(addr);
    nrf24_open_rx_pipe(0, addr);

    ce_high();


## **WARNING:** 
If you want to return register in it's own default value commenting function, with which set
value, and after compiling code does not solves this problem because nrf24 memorises most of 
register configurations even if you power off module. Therefore you must write even default 
values in this register.

For example if you had enabled crc with "nrf24_set_crc(en_crc, _2byte)" and you want to 
disable at begining of program you should write new values with "nrf24_set_crc(no_crc, _1byte)"
because as i mentioned nrf24 does not returns in default values even after powering off.




## **Enable/disable DPL:**

### **enable:**

    nrf24_dpl(enable);

    nrf24_set_rx_dpl(pipe, enable);


### **disable:**

    nrf24_dpl(disable);
    
    nrf24_set_rx_dpl(pipe, disable);



## **Enable/disable AUTO_ACK:**

### **enable:**

    nrf24_auto_ack_all(auto_ack);

### **disable:**

    nrf24_auto_ack_all(no_auto_ack); 


## **Enabled/disabled auto_ack on each pipe individualy:**

### **enable:**

    nrf24_auto_ack(pipe, auto_ack);

### **disable:**

    nrf24_auto_ack(pipe, no_auto_ack);

    nrf24_auto_retr_delay(0);
    nrf24_auto_retr_limit(15);
    


## **Enable/disable ACK with payload:**

### **enable:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(enable);

    nrf24_auto_retr_delay(0);
    nrf24_auto_retr_limit(15);


### **disable:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(disable);

    nrf24_auto_retr_delay(0);
    nrf24_auto_retr_limit(15);




## **Enable/disable and configure CRC:**

    nrf24_set_crc(en_crc, _1byte);

    //First argument can be set as en_crc, which enables CRC, or no_crc, which disables CRC.
    //As second argument can be used _1byte or _2byte.



## **Set TX RF power:**

    nrf24_tx_pwr(_0dbm);

    //Can be set used:
    //    n18dbm = 0 -> MIN
    //    n12dbm = 1
    //    n6dbm  = 2
    //    _0dbm  = 3 -> MAX



## **Set data rate:**

    nrf24_data_rate(_250kbps);

    //Can be set used:
    //    _1mbps   = 0
    //    _2mbps   = 1
    //    _250kbps = 2



## **Set channel:**

    nrf24_set_channel(90);
    
    //Can be used from 0 to 126 which is equivalent of 2400mhz to 2525mhz.
    //1 = 1mhz



## **Set payload size on each pipe:**

    nrf24_pipe_pld_size(pipe, payload_size);

    //pipe can be from 0 to 5

    //payload_size can be from 1 to 32



## **Set Delay betwen Auto-retransmissions:**

    nrf24_auto_retr_delay(0);
    
    //Can be set from 0 to 15.



## **Set Auto-retransmissions limit:**

    nrf24_auto_retr_limit(15);

    //Can be set from 0 to 15




## **Transmit:**

### **1.Without ack payload:**

    uint8_t dataT[PLD_SIZE] = {"Hello"};
    nrf24_transmit(dataT, sizeof(dataT));



### **2.With ack payload:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(enable);
    
    //and other configurations as well

    uint8_t dataT[PLD_SIZE] = {"Hello"};
    uint8_t tx_ack_pld[PLD_SIZE];

    uint8_t val = nrf24_transmit(dataT, sizeof(dataT));

    if(val == 0){
    	nrf24_receive(tx_ack_pld, sizeof(tx_ack_pld));
    }



### **3.With NO_ACK command:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(enable);
    nrf24_en_dyn_ack(enable);
    
    //and other configurations as well

    uint8_t dataT[PLD_SIZE] = {"Hello"};
    nrf24_transmit_no_ack(dataT, sizeof(dataT));


### **4.With ACK_PLD and IRQ**

    nrf24_transmit(dataT, sizeof(dataT));

    if(irq == 1){
	uint8_t stat = nrf24_r_status();

	if(stat & (1 << TX_DS)){
		nrf24_receive(tx_ack_pld, sizeof(tx_ack_pld));
	}else if(stat & (1 << MAX_RT)){
		nrf24_flush_tx();
		nrf24_clear_rx_dr();
	}
	irq = 0;
    }


## **Receive:**

### **1,Without ack payload:**

    uint8_t dataR[PLD_SIZE];

    nrf24_listen();

    if(nrf24_data_available()){
       nrf24_receive(dataR, sizeof(dataR));
    }



### **2.With transmit ack payload:**

    nrf24_auto_ack_all(auto_ack);
    nrf24_en_ack_pld(enable);
    
    //and other configurations as well


    uint8_t dataR[PLD_SIZE];
    uint8_t rx_ack_pld[PLD_SIZE] = {"OK"}

    nrf24_listen();

    if(nrf24_data_available()){
	nrf24_receive(dataR, sizeof(dataR));
	nrf24_transmit_rx_ack_pld(0, rx_ack_pld, sizeof(rx_ack_pld));
    }



### **3.With ACK_PLD and IRQ**

    if(irq == 1){
	uint8_t stat = nrf24_r_status();

	if(stat & (1 << RX_DR)){ 
		nrf24_receive(dataR, sizeof(dataR));
		nrf24_transmit_rx_ack_pld(0, rx_ack_pld, sizeof(rx_ack_pld));
	}
	irq = 0;
    }


    
