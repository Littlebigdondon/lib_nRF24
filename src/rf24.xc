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
static uint8_t get_channel(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return read_register(i_spi, spi_index, RF_CH);
}
static uint8_t spi_transfer(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t command) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    uint8_t status = i_spi.transfer8(command);
    i_spi.end_transaction(DEASSERT_TICKS);
    return status;
}
static uint8_t flush_rx(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return spi_transfer(i_spi, spi_index, FLUSH_RX);
}
static uint8_t flush_tx(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return spi_transfer(i_spi, spi_index, FLUSH_TX);
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
static rf24_datarate_e get_data_rate(client interface spi_master_if i_spi,
        unsigned spi_index) {
    rf24_datarate_e result;
    uint8_t dr = read_register(i_spi, spi_index, RF_SETUP)
            & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    if (dr == _BV(RF_DR_LOW))
        result = RF24_250KBPS;
    else if (dr == _BV(RF_DR_HIGH))
        result = RF24_2MBPS;
    else
        result = RF24_1MBPS;
    return result;
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
static void read(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t *buf, uint8_t len) {
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
}
static void start_fast_write(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce,
        const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast, rf24_bool_e start_tx) {
    write_payload(
            i_spi,
            spi_index,
            buf,
            len,
            multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD
            );
    if (start_tx) {
        p_ce <: HIGH;
    }
}
static rf24_bool_e write(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce,
        const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast, rf24_bool_e start_tx) {
    start_fast_write(
            i_spi,
            spi_index,
            p_ce,
            buf,
            len,
            multicast,
            start_tx
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
        return FALSE;
    }
    return TRUE;
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
static rf24_bool_e available(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t *pipe_num) {
    if (!(read_register(i_spi, spi_index, FIFO_STATUS) & _BV(RX_EMPTY))) {
        if (pipe_num) {
            uint8_t status = get_status(i_spi, spi_index);
            *pipe_num = (status >> RX_P_NO) & 0x07;
        }
        return TRUE;
    } else {
        return FALSE;
    }
}
static uint8_t write_fast(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce, const uint8_t *buf, uint8_t len,
        const rf24_bool_e multicast) {
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high
    while (get_status(i_spi, spi_index) & _BV(TX_FULL)) {
        if (get_status(i_spi, spi_index) & _BV(MAX_RT)) {
            write_register(i_spi, spi_index, NRF_STATUS, _BV(MAX_RT));
            return 0;
        }
    }
    start_fast_write(i_spi, spi_index, p_ce, buf, len, multicast, FALSE);
    return 1;
}
static void stop_listening(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce) {
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
}
static void start_listening(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce) {
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
}
static rf24_bool_e tx_standby(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce,
        uint32_t timeout, rf24_bool_e start_tx) {
    uint32_t start, timeout_ticks;
    timer t;

    if (start_tx) {
        stop_listening(i_spi, spi_index, p_ce);
        p_ce <: HIGH;
    }

    if (timeout) {
        t :> start;
        // Convert ms to 100MHz timer ticks
        timeout_ticks = start + (timeout * 100000);
    }

    while (!(read_register(i_spi, spi_index, FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (get_status(i_spi, spi_index) & _BV(MAX_RT)) {
            write_register(i_spi, spi_index, NRF_STATUS, _BV(MAX_RT));
            p_ce <: LOW;
            p_ce <: HIGH;
            if (timeout) {
                select {
                    case t when timerafter(timeout_ticks) :> void:
                        p_ce <: LOW;
                        flush_tx(i_spi, spi_index);
                        return FALSE;
                    default:
                        break;
                }
            }
        }
    }

    //Set STANDBY-I mode
    p_ce <: LOW;

    return TRUE;
}
static void write_ack_payload(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t pipe,
        const uint8_t *buf, uint8_t len) {
    uint8_t data_len = rf24_min(len, 32);

    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(W_ACK_PAYLOAD | (pipe & 0x07));
    while (data_len--)
        i_spi.transfer8(*buf++);
    i_spi.end_transaction(DEASSERT_TICKS);
}
static rf24_bool_e is_ack_payload_available(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return !(read_register(i_spi, spi_index, FIFO_STATUS) & _BV(RX_EMPTY));
}
{rf24_bool_e, rf24_bool_e, rf24_bool_e} what_happened(client interface spi_master_if i_spi,
        unsigned spi_index) {
    uint8_t status = write_register(
            i_spi,
            spi_index,
            NRF_STATUS,
            _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)
            );
    return {status & _BV(TX_DS), status & _BV(MAX_RT), status & _BV(RX_DR)};
}
static void reuse_tx(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce) {
    // Clear max retry flag
    write_register(i_spi, spi_index, NRF_STATUS, _BV(MAX_RT));
    // Re-transfer packet
    spi_transfer(i_spi, spi_index, REUSE_TX_PL);
    p_ce <: LOW;
    p_ce <: HIGH;
}
static rf24_bool_e test_carrier(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return (read_register(i_spi, spi_index, CD) & 1);
}
static rf24_bool_e test_RPD(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return (read_register(i_spi, spi_index, RPD) & 1);
}
static void set_address_width(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t width) {
    if ((width -= 2)) {
        write_register(i_spi, spi_index, SETUP_AW, width % 4);
        addr_width = (width % 4) + 2;
    } else {
        write_register(i_spi, spi_index, SETUP_AW, 0);
        addr_width = 2;
    }
}
static void set_payload_size(uint8_t size) {
    payload_size = rf24_min(size, 32);
}
static uint8_t get_payload_size() {
    return payload_size;
}
static uint8_t get_dynamic_payload_size(client interface spi_master_if i_spi,
        unsigned spi_index) {
    i_spi.begin_transaction(spi_index, SPEED_KHZ, SPI_MODE);
    i_spi.transfer8(R_RX_PL_WID);
    uint8_t result = i_spi.transfer8(0xFF);
    i_spi.end_transaction(DEASSERT_TICKS);
    if (result > 32) {
        flush_rx(i_spi, spi_index);
        delay_milliseconds(2);
        return 0;
    }
    return result;
}
static void enable_ack_payload(client interface spi_master_if i_spi,
        unsigned spi_index) {
    // enable ack payload and dynamic payload features
    write_register(
            i_spi,
            spi_index,
            FEATURE,
            read_register(
                    i_spi,
                    spi_index,
                    FEATURE
                    ) | _BV(EN_ACK_PAY) | _BV(EN_DPL)
            );
    // Enable dynamic payload on pipes 0 & 1
    write_register(
            i_spi,
            spi_index,
            DYNPD,
            read_register(
                    i_spi,
                    spi_index,
                    DYNPD
                    ) | _BV(DPL_P1) | _BV(DPL_P0)
            );
    dynamic_payloads_enabled = TRUE;
}
static void enable_dynamic_payloads(client interface spi_master_if i_spi,
        unsigned spi_index) {
    // Enable dynamic payload throughout the system
    write_register(
            i_spi,
            spi_index,
            FEATURE,
            read_register(
                    i_spi,
                    spi_index,
                    FEATURE
                    ) | _BV(EN_DPL)
            );
    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register(
            i_spi,
            spi_index,
            DYNPD,
            read_register(
                    i_spi,
                    spi_index,
                    DYNPD
                    )
                    | _BV(DPL_P5) | _BV(DPL_P4)
                    | _BV(DPL_P3) | _BV(DPL_P2)
                    | _BV(DPL_P1) | _BV(DPL_P0)
            );
    dynamic_payloads_enabled = TRUE;
}
static void disable_dynamic_payloads(client interface spi_master_if i_spi,
        unsigned spi_index) {
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads
    write_register(i_spi, spi_index, FEATURE, 0);
    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register(i_spi, spi_index, DYNPD, 0);
    dynamic_payloads_enabled = FALSE;
}
static void enable_dynamic_ack(client interface spi_master_if i_spi,
        unsigned spi_index) {
    // Enable dynamic ack features
    write_register(
            i_spi,
            spi_index,
            FEATURE,
            read_register(
                    i_spi,
                    spi_index,
                    FEATURE
                    ) | _BV(EN_DYN_ACK)
            );
}
static rf24_bool_e is_p_variant() {
    return p_variant;
}
static void set_auto_ack(client interface spi_master_if i_spi,
        unsigned spi_index, rf24_bool_e enable) {
    if (enable)
        write_register(i_spi, spi_index, EN_AA, 0x3F);
    else
        write_register(i_spi, spi_index, EN_AA, 0x00);
}
static void set_auto_ack_on_pipe(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t pipe, rf24_bool_e enable) {
    if (pipe < 6) {
        uint8_t en_aa = read_register(i_spi, spi_index, EN_AA);
        if (enable)
            en_aa |= _BV(pipe);
        else
            en_aa &= ~_BV(pipe);
        write_register(i_spi, spi_index, EN_AA, en_aa);
    }
}
static void set_PA_level(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t level) {
    uint8_t setup = read_register(i_spi, spi_index, RF_SETUP) & 0xF8;
    // If invalid level, go to max PA
    // +1 to support the SI24R1 chip extra bit
    if (level > 3)
        level = (RF24_PA_MAX << 1) + 1;
    // Else set level as requested
    else
        level = (level << 1) + 1;
    write_register(i_spi, spi_index, RF_SETUP, setup |= level);
}
static uint8_t get_PA_level(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return (read_register(i_spi, spi_index, RF_SETUP)
            & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}
static void set_crc_length(client interface spi_master_if i_spi,
        unsigned spi_index, rf24_crclength_e length) {
    uint8_t config = read_register(i_spi, spi_index, NRF_CONFIG)
            & ~(_BV(CRCO) | _BV(EN_CRC));
    if (length == RF24_CRC_8)
        config |= _BV(EN_CRC);
    else if (length != RF24_CRC_DISABLED) {
        config |= _BV(EN_CRC);
        config |= _BV(CRCO);
    }
    // If length == RF24_CRC_DISABLED, do nothing
    // because we turned it off already, above ^^
    write_register(i_spi, spi_index, NRF_CONFIG, config);
}
static rf24_crclength_e get_crc_length(client interface spi_master_if i_spi,
        unsigned spi_index) {
    rf24_crclength_e result = RF24_CRC_DISABLED;

    uint8_t config = read_register(i_spi, spi_index, NRF_CONFIG)
            & (_BV(CRCO) | _BV(EN_CRC));
    uint8_t AA = read_register(i_spi, spi_index, EN_AA);

    if (config & _BV(EN_CRC) || AA) {
        if (config & _BV(CRCO))
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }
    return result;
}
static void disable_crc(client interface spi_master_if i_spi,
        unsigned spi_index) {
    uint8_t disable = read_register(i_spi, spi_index, NRF_CONFIG)
            & ~_BV(EN_CRC);
    write_register(i_spi, spi_index, NRF_CONFIG, disable);
}
static void mask_irq(client interface spi_master_if i_spi,
        unsigned spi_index, rf24_bool_e tx_ok,
        rf24_bool_e tx_fail, rf24_bool_e rx_ready) {
    uint8_t config = read_register(i_spi, spi_index, NRF_CONFIG);
    // Clear the interrupt flags
    config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
    // Set the specified interrupt flags
    config |= tx_fail << MASK_MAX_RT | tx_ok << MASK_TX_DS | rx_ready << MASK_RX_DR;
    write_register(i_spi, spi_index, NRF_CONFIG, config);
}
static rf24_bool_e begin(client interface spi_master_if i_spi,
        unsigned spi_index, out port p_ce){
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
    return (setup != 0 && setup != 0xFF);
}
static rf24_bool_e is_chip_connected(client interface spi_master_if i_spi,
        unsigned spi_index) {
    uint8_t setup = read_register(i_spi, spi_index, SETUP_AW);
    return (setup >= 1 && setup <= 3);
}
static void open_writing_pipe(client interface spi_master_if i_spi,
        unsigned spi_index, const uint8_t *address) {
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
}
static void open_reading_pipe(client interface spi_master_if i_spi,
        unsigned spi_index, uint8_t child, const uint8_t *address) {
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
}
static rf24_bool_e rx_fifo_full(client interface spi_master_if i_spi,
        unsigned spi_index) {
    return (read_register(i_spi, spi_index, FIFO_STATUS) & _BV(RX_FULL));
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
                r = begin(i_spi, spi_index, p_ce);
                break;
            case i_rf24.is_chip_connected() -> rf24_bool_e r:
                r = is_chip_connected(i_spi, spi_index);
                break;
            case i_rf24.start_listening():
                start_listening(i_spi, spi_index, p_ce);
                break;
            case i_rf24.stop_listening():
                stop_listening(i_spi, spi_index, p_ce);
                break;
            case i_rf24.available(uint8_t *pipe_num) -> rf24_bool_e r:
                r = available(i_spi, spi_index, pipe_num);
                break;
            case i_rf24.read(uint8_t *buf, uint8_t len):
                    read(i_spi, spi_index, buf, len);
                break;
            case i_rf24.write(const uint8_t *buf,
                    uint8_t len, const rf24_bool_e multicast) -> rf24_bool_e r:
                r = write(
                        i_spi,
                        spi_index,
                        p_ce,
                        buf,
                        len,
                        multicast,
                        TRUE
                        );
                break;
            case i_rf24.open_writing_pipe(const uint8_t *address):
                    open_writing_pipe(i_spi, spi_index, address);
                break;
            case i_rf24.open_reading_pipe(uint8_t child, const uint8_t *address):
                open_reading_pipe(i_spi, spi_index, child, address);
                break;
            case i_rf24.rx_fifo_full() -> rf24_bool_e r:
                r = rx_fifo_full(i_spi, spi_index);
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
            case i_rf24.tx_standby() -> rf24_bool_e r:
                r = tx_standby(
                        i_spi,
                        spi_index,
                        p_ce,
                        0,
                        FALSE
                        );
                break;
            case i_rf24.tx_standby_with_timeout(uint32_t timeout,
                    rf24_bool_e start_tx) -> rf24_bool_e r:
                r = tx_standby(
                        i_spi,
                        spi_index,
                        p_ce,
                        timeout,
                        start_tx
                        );
                break;
            case i_rf24.write_ack_payload(uint8_t pipe,
                    const uint8_t *buf, uint8_t len):
                write_ack_payload(
                        i_spi,
                        spi_index,
                        pipe,
                        buf,
                        len
                        );
                break;
            case i_rf24.is_ack_payload_available() -> rf24_bool_e r:
                r = is_ack_payload_available(i_spi, spi_index);
                break;
            case i_rf24.what_happened() -> {rf24_bool_e tx_ok,
                    rf24_bool_e tx_fail, rf24_bool_e rx_ready}:
                {tx_ok, tx_fail, rx_ready} = what_happened(i_spi, spi_index);
                break;
            case i_rf24.start_fast_write(const uint8_t *buf, uint8_t len,
                    const rf24_bool_e multicast, rf24_bool_e start_tx):
                start_fast_write(
                        i_spi,
                        spi_index,
                        p_ce,
                        buf,
                        len,
                        multicast,
                        start_tx
                        );
                break;
            case i_rf24.start_write(const uint8_t *buf, uint8_t len,
                    const rf24_bool_e multicast):
                start_write(
                        i_spi,
                        spi_index,
                        p_ce,
                        buf,
                        len,
                        multicast
                        );
                break;
            case i_rf24.reuse_tx():
                reuse_tx(i_spi, spi_index, p_ce);
                break;
            case i_rf24.flush_tx() -> uint8_t r:
                r = flush_tx(i_spi, spi_index);
                break;
            case i_rf24.flush_rx() -> uint8_t r:
                r = flush_rx(i_spi, spi_index);
                break;
            case i_rf24.test_carrier() -> rf24_bool_e r:
                r = test_carrier(i_spi, spi_index);
                break;
            case i_rf24.test_RPD() -> rf24_bool_e r:
                r = test_RPD(i_spi, spi_index);
                break;
            case i_rf24.close_reading_pipe(uint8_t pipe):
                close_reading_pipe(i_spi, spi_index, pipe);
                break;
            case i_rf24.set_address_width(uint8_t width):
                set_address_width(i_spi, spi_index, width);
                break;
            case i_rf24.set_retries(uint8_t delay, uint8_t count):
                set_retries(i_spi, spi_index, delay, count);
                break;
            case i_rf24.set_channel(uint8_t channel):
                set_channel(i_spi, spi_index, channel);
                break;
            case i_rf24.get_channel() -> uint8_t r:
                r = get_channel(i_spi, spi_index);
                break;
            case i_rf24.set_payload_size(uint8_t size):
                set_payload_size(size);
                break;
            case i_rf24.get_payload_size() -> uint8_t r:
                r = get_payload_size();
                break;
            case i_rf24.get_dynamic_payload_size() -> uint8_t r:
                r = get_dynamic_payload_size(i_spi, spi_index);
                break;
            case i_rf24.enable_ack_payload():
                enable_ack_payload(i_spi, spi_index);
                break;
            case i_rf24.enable_dynamic_payloads():
                enable_dynamic_payloads(i_spi, spi_index);
                break;
            case i_rf24.disable_dynamic_payloads():
                disable_dynamic_payloads(i_spi, spi_index);
                break;
            case i_rf24.enable_dynamic_ack():
                enable_dynamic_ack(i_spi, spi_index);
                break;
            case i_rf24.is_p_variant() -> rf24_bool_e r:
                r = is_p_variant();
                break;
            case i_rf24.set_auto_ack(rf24_bool_e enable):
                set_auto_ack(i_spi, spi_index, enable);
                break;
            case i_rf24.set_auto_ack_on_pipe(uint8_t pipe,
                    rf24_bool_e enable):
                set_auto_ack_on_pipe(i_spi, spi_index, pipe, enable);
                break;
            case i_rf24.set_PA_level(uint8_t level):
                set_PA_level(i_spi, spi_index, level);
                break;
            case i_rf24.get_PA_level() -> uint8_t r:
                r = get_PA_level(i_spi, spi_index);
                break;
            case i_rf24.set_data_rate(rf24_datarate_e speed)
                    -> rf24_bool_e r:
                r = set_data_rate(i_spi, spi_index, speed);
                break;
            case i_rf24.get_data_rate() -> rf24_datarate_e r:
                r = get_data_rate(i_spi, spi_index);
                break;
            case i_rf24.set_crc_length(rf24_crclength_e length):
                set_crc_length(i_spi, spi_index, length);
                break;
            case i_rf24.get_crc_length() -> rf24_crclength_e r:
                r = get_crc_length(i_spi, spi_index);
                break;
            case i_rf24.disable_crc():
                disable_crc(i_spi, spi_index);
                break;
            case i_rf24.mask_irq(rf24_bool_e tx_ok,
                    rf24_bool_e tx_fail, rf24_bool_e rx_ready):
                mask_irq(i_spi, spi_index, tx_ok, tx_fail, rx_ready);
                break;
        }
    }
}
