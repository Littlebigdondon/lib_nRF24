/*
 * rf24.xc
 *
 *  Created on: Oct 8, 2018
 *      Author: Donnie Pitts
 */

#include "rf24.h"
#include <string.h>

rf24_bool_e p_variant = FALSE; /* False for RF24L01 and true for RF24L01P */
uint8_t payload_size; /**< Fixed size of payloads */
rf24_bool_e dynamic_payloads_enabled = FALSE; /**< Whether dynamic payloads are enabled. */
uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */

static const uint8_t child_pipe_enable[] = {
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};
static const uint8_t child_pipe[] = {
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] = {
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

static uint8_t read_register(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t reg) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    uint8_t result = i_spi.transfer8(R_REGISTER | (REGISTER_MASK & reg));
    i_spi.end_transaction(DEASSERT_TICKS);
    return result;
}
static uint8_t write_register(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t reg, uint8_t value) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    uint8_t status = i_spi.transfer8(W_REGISTER | (REGISTER_MASK & reg));
    i_spi.transfer8(value);
    i_spi.end_transaction(DEASSERT_TICKS);
    return status;
}
static void write_register_buffered(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t reg, const uint8_t *buf, uint8_t len) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(W_REGISTER | (REGISTER_MASK & reg));
    while (len--)
        i_spi.transfer8(*buf++);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static void set_retries(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t delay, uint8_t count) {
    write_register(
            i_spi,
            spi_index,
            SETUP_RETR,
            (delay & 0xF) << ARD | (count & 0xF) << ARC
            );
}
static void toggle_features(client interface spi_master_if i_spi,
        unsigned spi_index) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(ACTIVATE);
    i_spi.transfer8(0x73);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static void set_channel(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t channel) {
    const uint8_t max_channel = 125;
    write_register(i_spi, spi_index, RF_CH, rf24_min(channel, max_channel));
}
static uint8_t spi_transfer(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t command) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    uint8_t status = i_spi.transfer8(command);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static void flush_rx(client interface spi_master_if i_spi,
        unsigned spi_index) {
    spi_transfer(i_spi, spi_index, FLUSH_RX);
}
static void flush_tx(client interface spi_master_if i_spi,
        unsigned spi_index) {
    spi_transfer(i_spi, spi_index, FLUSH_TX);
}
static void power_up(client interface spi_master_if i_spi,
        unsigned spi_index) {
    uint8_t cfg = read_register(i_spi, spi_index, NRF_CONFIG);
    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))){
       write_register(i_spi, spi_index, NRF_CONFIG, cfg | _BV(PWR_UP));

       // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
       // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
       // the CE is set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
       delay_milliseconds(5);
    }
}
static void power_down(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce) {
    p_ce <: LOW;
    write_register(
            i_spi,
            spi_index,
            NRF_CONFIG,
            read_register(
                    i_spi,
                    spi_index,
                    NRF_CONFIG
                    ) & ~_BV(PWR_UP)
            );
}
static void enable_ptx(client interface spi_master_if i_spi,
        unsigned spi_index) {
    write_register(
            i_spi,
            spi_index,
            NRF_CONFIG,
            read_register(
                    i_spi,
                    spi_index,
                    NRF_CONFIG
                    ) & ~_BV(PRIM_RX)
            );
}
static rf24_bool_e set_data_rate(client interface spi_master_if i_spi,
        unsigned spi_index, rf24_datarate_e speed) {
    uint8_t setup = read_register(i_spi, spi_index, RF_SETUP);
    // HIGH and LOW '00' is 1Mbs - our default
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    switch (speed) {
    case RF24_250KBPS:
        setup |= _BV(RF_DR_LOW);
        break;
    case RF24_2MBPS:
        setup |= _BV(RF_DR_HIGH);
        break;
    }

    write_register(i_spi, spi_index, RF_SETUP, setup);
    // Verify our result
    return (read_register(i_spi, spi_index, RF_SETUP) == setup);
}
static void close_reading_pipe(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t pipe) {
    write_register(
            i_spi,
            spi_index,
            EN_RXADDR,
            read_register(
                    i_spi,
                    spi_index,
                    EN_RXADDR
                    ) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe]))
            );
}
static void read_payload(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t *buf, uint8_t data_len) {
    data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(R_RX_PAYLOAD);
    while (data_len--) {
        *buf = i_spi.transfer8(0xFF);
        *buf++;
    }
    while (blank_len--)
        i_spi.transfer8(0xFF);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static void write_payload(client interface spi_master_if i_spi,
        unsigned spi_index,
        const uint8_t *buf, uint8_t data_len,
        const uint8_t write_type) {
    data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(R_RX_PAYLOAD);
    while (data_len--) {
        i_spi.transfer8(*buf);
        *buf++;
    }
    while (blank_len--)
        i_spi.transfer8(0);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static void start_fast_write(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce,
        const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast) {
    write_payload(
            i_spi,
            spi_index,
            buf,
            len,
            multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD
            );
    // Used to be a parameter, but no function in the API calls with
    // "start_tx" at all. So I'm just gonna leave this out
//    if (start_tx) {
//        p_ce <: HIGH;
//    }
}
static void write(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce,
        const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast, rf24_bool_e start_tx) {
    start_fast_write(
            i_spi,
            spi_index,
            p_ce,
            buf,
            len,
            multicast
            );
    p_ce <: LOW;
    uint8_t status = write_register(
            i_spi,
            spi_index,
            NRF_STATUS,
            _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)
            );
    if (status & _BV(MAX_RT)) {
        flush_tx(i_spi, spi_index);
    }
}
static void start_write(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce, const uint8_t *buf,
        uint8_t len, const rf24_bool_e multicast) {
    write_payload(
            i_spi,
            spi_index,
            buf,
            len,
            multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD
            );
    p_ce <: HIGH;
    delay_microseconds(10);
    p_ce <: LOW;
}
static uint8_t get_status(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return spi_transfer(i_spi, spi_index, RF24_NOP);
}
static uint8_t write_fast(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce, const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast) {
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    // TODO: implement timeout
    while (get_status(i_spi, spi_index) & _BV(TX_FULL)) {
        if (get_status(i_spi, spi_index) & _BV(MAX_RT)) {
            write_register(i_spi, spi_index, NRF_STATUS, _BV(MAX_RT));
            return 0;
        }
    }
    start_fast_write(i_spi, spi_index, p_ce, buf, len, multicast);
    return 1;
}

void RF24(
        server rf24_if i_rf24,
        client interface spi_master_if i_spi,
        unsigned spi_index,
        out port p_ce,
        in port ?p_irq
        ) {

    while (1) {
        select {
            case i_rf24.begin() -> rf24_bool_e r:
                p_ce <: LOW;
                i_spi.end_transaction(DEASSERT_TICKS);
                // Must allow the radio time to settle else configuration bits will not necessarily stick.
                // This is actually only required following power up but some settling time also appears to
                // be required after resets too. For full coverage, we'll always assume the worst.
                // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
                // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
                // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
                delay_milliseconds(5);
                // Reset NRF_CONFIG and enable 16-bit CRC.
                write_register(i_spi, spi_index, NRF_CONFIG, 0x0C);
                // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
                // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
                // sizes must never be used. See documentation for a more complete explanation.
                set_retries(i_spi, spi_index, 5, 15);
                // check for connected module and if this is a p nRF24l01 variant
                if(set_data_rate(i_spi, spi_index, RF24_250KBPS)) {
                    p_variant = TRUE;
                }
                uint8_t setup = read_register(i_spi, spi_index, RF_SETUP);
                // Then set the data rate to the slowest (and most reliable) speed supported by all hardware.
                set_data_rate(i_spi, spi_index, RF24_1MBPS);
                // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
                toggle_features(i_spi, spi_index);
                write_register(i_spi, spi_index, FEATURE, 0);
                write_register(i_spi, spi_index, DYNPD, 0);
                dynamic_payloads_enabled = FALSE;
                // Reset current status
                write_register(
                        i_spi,
                        spi_index,
                        NRF_STATUS,
                        _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)
                        );
                // Set up default configuration.  Callers can always change it later.
                // This channel should be universally safe and not bleed over into adjacent
                // spectrum.
                set_channel(i_spi, spi_index, 76);
                // Flush buffers
                flush_rx(i_spi, spi_index);
                flush_tx(i_spi, spi_index);
                //Power up by default when begin() is called
                power_up(i_spi, spi_index);
                // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
                // PTX should use only 22uA of power
                enable_ptx(i_spi, spi_index);
                r = (rf24_bool_e)(setup != 0 && setup != 0xFF);
                break;
            case i_rf24.is_chip_connected() -> rf24_bool_e r:
                uint8_t setup = read_register(i_spi, spi_index, SETUP_AW);
                r = (setup >= 1 && setup <= 3);
                break;
            case i_rf24.start_listening():
                power_up(i_spi, spi_index);
                write_register(
                        i_spi,
                        spi_index,
                        NRF_CONFIG,
                        read_register(
                                i_spi,
                                spi_index,
                                NRF_CONFIG
                                ) | _BV(PRIM_RX)
                        );
                write_register(
                        i_spi,
                        spi_index,
                        NRF_STATUS,
                        _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)
                        );
                p_ce <: HIGH;
                // Restore the pipe0 address, if it exists
                if (pipe0_reading_address[0] > 0) {
                    write_register_buffered(
                            i_spi,
                            spi_index,
                            RX_ADDR_P0,
                            pipe0_reading_address,
                            addr_width
                            );
                } else {
                    close_reading_pipe(i_spi, spi_index, 0);
                }
                break;
            case i_rf24.stop_listening():
                p_ce <: LOW;
                delay_microseconds(200);
                if (read_register(i_spi, spi_index, FEATURE) & _BV(EN_ACK_PAY)) {
                    delay_microseconds(200);
                    flush_tx(i_spi, spi_index);
                }
                write_register(
                        i_spi,
                        spi_index,
                        NRF_CONFIG,
                        read_register(i_spi, spi_index, NRF_CONFIG) & ~_BV(PRIM_RX)
                        );
                write_register(
                        i_spi,
                        spi_index,
                        EN_RXADDR,
                        read_register(
                                i_spi,
                                spi_index,
                                EN_RXADDR
                                ) | _BV(pgm_read_byte(&child_pipe_enable[0]))
                        );
                break;
            case i_rf24.available(uint8_t *pipe_num) -> rf24_bool_e r:
                if (!(read_register(i_spi, spi_index, FIFO_STATUS) & _BV(RX_EMPTY))) {
                    if (pipe_num) {
                        uint8_t status = get_status(i_spi, spi_index);
                        *pipe_num = (status >> RX_P_NO) & 0x07;
                    }
                    r = TRUE;
                } else {
                    r = FALSE;
                }
                break;
            case i_rf24.read(uint8_t *buf, uint8_t len):
                // Fetch the payload
                read_payload(
                        i_spi,
                        spi_index,
                        buf,
                        len
                        );
                // Clear the two possible interrupt flags with one command
                write_register(
                        i_spi,
                        spi_index,
                        NRF_STATUS,
                        _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS)
                        );
                break;
            case i_rf24.write(const uint8_t *buf,
                    uint8_t len, const rf24_bool_e multicast) -> rf24_bool_e r:
                write(
                        i_spi,
                        spi_index,
                        p_ce,
                        buf,
                        len,
                        multicast,
                        TRUE
                        );
                r = TRUE;
                break;
            case i_rf24.open_writing_pipe(const uint8_t *address):
                write_register_buffered(
                        i_spi,
                        spi_index,
                        RX_ADDR_P0,
                        address,
                        addr_width
                        );
                write_register_buffered(
                        i_spi,
                        spi_index,
                        TX_ADDR,
                        address,
                        addr_width
                        );
                write_register(
                        i_spi,
                        spi_index,
                        RX_PW_P0,
                        payload_size
                        );
                break;
            case i_rf24.open_reading_pipe(uint8_t child, const uint8_t *address):
                if (child == 0)
                    memcpy(pipe0_reading_address, address, addr_width);
                if (child < 6) {
                    // For pipes 2-5, only write the LSB
                    if (child < 2) {
                        write_register_buffered(
                                i_spi,
                                spi_index,
                                pgm_read_byte(&child_pipe[child]),
                                address,
                                addr_width
                                );
                    } else {
                        write_register_buffered(
                                i_spi,
                                spi_index,
                                pgm_read_byte(&child_pipe[child]),
                                address,
                                1
                                );
                    }
                    write_register(
                            i_spi,
                            spi_index,
                            pgm_read_byte(&child_payload_size[child]),
                            payload_size
                            );
                    write_register(
                            i_spi,
                            spi_index,
                            EN_RXADDR,
                            read_register(
                                    i_spi,
                                    spi_index,
                                    EN_RXADDR
                                    ) | _BV(pgm_read_byte(&child_pipe_enable[child]))
                            );
                }
                break;
            case i_rf24.rx_fifo_full() -> rf24_bool_e r:
                r = read_register(
                        i_spi,
                        spi_index,
                        FIFO_STATUS
                        ) & _BV(RX_FULL);
                break;
            case i_rf24.power_up():
                power_up(i_spi, spi_index);
                break;
            case i_rf24.power_down():
                power_down(i_spi, spi_index, p_ce);
                break;
            case i_rf24.write_fast(const uint8_t *buf,
                    uint8_t len, const rf24_bool_e multicast) -> rf24_bool_e r:
                r = write_fast(
                        i_spi,
                        spi_index,
                        p_ce,
                        buf,
                        len,
                        multicast
                        );
                break;
        }
    }
}
