/*
 * rf24.h
 *
 *  Created on: Oct 8, 2018
 *      Author: Donnie
 */


#ifndef RF24_H_
#define RF24_H_
#include <xs1.h>
#include <stdint.h>
#include <spi.h>
#include <gpio.h>
#include "nRF24L01.h"

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;


typedef enum { FALSE = 0, TRUE } rf24_bool_e;

#define LOW 0
#define HIGH 1

#define SPEED_KHZ 4000
#define SPI_MODE SPI_MODE_0
#define DEASSERT_TICKS 0
#define _BV(x) (1<<(x))
#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

typedef interface rf24_if {
    // Primary Interface
    rf24_bool_e begin();
    rf24_bool_e is_chip_connected();
    void start_listening();
    void stop_listening();
//    rf24_bool_e available();
    [[clears_notification]] void read(uint8_t *buf, uint8_t len);
    [[clears_notification]] rf24_bool_e write(const uint8_t *buf,
            uint8_t len, const rf24_bool_e multicast);
    void open_writing_pipe(const uint8_t *address);
    void open_reading_pipe(uint8_t number, const uint8_t *address);

    // Advanced Operations
//    void printDetails(); // TODO
    rf24_bool_e available(uint8_t *pipe_num);
    rf24_bool_e rx_fifo_full();
    void power_down();
    void power_up();
//    rf24_bool_e write_fast(const void *buf, uint8_t len);
    rf24_bool_e write_fast(const uint8_t *buf, uint8_t len, const rf24_bool_e multicast);
//    rf24_bool_e write_blocking(const void *buf, uint8_t len, uint32_t timeout); // TODO
    rf24_bool_e tx_standby();
    rf24_bool_e tx_standby_with_timeout(uint32_t timeout, rf24_bool_e start_tx);
    void write_ack_payload(uint8_t pipe, const uint8_t *buf, uint8_t len);
    rf24_bool_e is_ack_payload_available();
    {rf24_bool_e, rf24_bool_e, rf24_bool_e} what_happened();
    void start_fast_write(const uint8_t *buf, uint8_t len, const rf24_bool_e multicast, rf24_bool_e start_tx);
    void start_write(const uint8_t *buf, uint8_t len, const rf24_bool_e multicast);
    void reuse_tx();
    uint8_t flush_tx();
    uint8_t flush_rx();
    rf24_bool_e test_carrier();
    rf24_bool_e test_RPD();
//    rf24_bool_e is_valid(); // TODO
    void close_reading_pipe(uint8_t pipe);
    void set_address_width(uint8_t width);
    void set_retries(uint8_t delay, uint8_t count);
    void set_channel(uint8_t channel);
    uint8_t get_channel();
    void set_payload_size(uint8_t size);
    uint8_t get_payload_size();
    uint8_t get_dynamic_payload_size();
    void enable_ack_payload();
    void enable_dynamic_payloads();
    void disable_dynamic_payloads();
    void enable_dynamic_ack();
    rf24_bool_e is_p_variant();
    void set_auto_ack(rf24_bool_e enable);
    void set_auto_ack_on_pipe(uint8_t pipe, rf24_bool_e enable);
    void set_PA_level(uint8_t level);
    uint8_t get_PA_level();
    rf24_bool_e set_data_rate(rf24_datarate_e speed);
    rf24_datarate_e get_data_rate();
    void set_crc_length(rf24_crclength_e length);
    rf24_crclength_e get_crc_length();
    void disable_crc();
    void mask_irq(rf24_bool_e tx_ok, rf24_bool_e tx_fail, rf24_bool_e rx_ready);
//    void open_reading_pipe(uint8_t number, uint64_t address); // TODO
//    void open_writing_pipe(uint64_t address); // TODO
    [[notification]] slave void interrupt();
    [[clears_notification]] void clear_interrupt();
} rf24_if;

[[distributable]]
void rf24_driver(
        server rf24_if i_rf24,
        client interface spi_master_if i_spi,
        unsigned spi_index,
        out port p_ce,
        client interface input_gpio_if ?i_irq
        );
#endif /* RF24_H_ */
