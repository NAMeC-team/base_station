/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "nrf24l01.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "radio_utils.h"
#include "ssl_data.pb.h"
#include "swo.h"

namespace {
#define HALF_PERIOD 500ms
}

EventQueue event_queue;

static DigitalOut led1(LED1);
static SPI spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
static NRF24L01 radio(&spi, DIO1, DIO2, NC);
static UnbufferedSerial serial_port(USBTX, USBRX);

static IAToMainBoard ai_message = IAToMainBoard_init_zero;
static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };
static uint8_t radio_packet[IAToMainBoard_size + 1];

using namespace sixtron;
SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

void send_packet(uint8_t *packet, size_t length)
{
    radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, length);
    radio.send_packet(packet, length);
}

void send_protobuf_packet()
{
    // printf("send_protobuf_packet\n");
    // radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, IAToMainBoard_size);
    // printf("0x");
    //     for (int i = 0; i < IAToMainBoard_size; i++) {
    //     printf("%02x", radio_packet[i]);
    // }
    // printf("\n");
    radio.send_packet(radio_packet, IAToMainBoard_size + 1);
}

void fill_radio_packet(packet_type_t type, com_packet_t *packet_to_encode)
{
    uint8_t length = 0;

    switch (type) {
        case PACKET_MASTER:
            // type field
            packet_to_encode->type = PACKET_MASTER;
            length++;
            // TODO: add payload
            packet_to_encode->master.actions = ACTION_ON;
            length++;
            packet_to_encode->master.x_speed = 4555;
            length = length + sizeof(packet_to_encode->master.x_speed);
            packet_to_encode->master.y_speed = 3000;
            length = length + sizeof(packet_to_encode->master.y_speed);
            packet_to_encode->master.t_speed = 4000;
            length = length + sizeof(packet_to_encode->master.t_speed);
            packet_to_encode->master.kickPower = 0x02;
            length++;
            break;
        case PACKET_PARAMS:
            // type field
            packet_to_encode->type = PACKET_PARAMS;
            length++;
            // TODO: add payload
            packet_to_encode->params.kp = 1.34;
            length = length + sizeof(packet_to_encode->params.kp);
            packet_to_encode->params.ki = 5.34;
            length = length + sizeof(packet_to_encode->params.ki);
            packet_to_encode->params.kd = 10.34;
            length = length + sizeof(packet_to_encode->params.kd);
            break;
        case PACKET_ROBOT:
            // type field
            packet_to_encode->type = PACKET_ROBOT;
            length++;
            // TODO: add payload
            packet_to_encode->robot.id = 0xAA;
            length++;
            packet_to_encode->robot.status = 0x00;
            length++;
            packet_to_encode->robot.cap_volt = 0x05;
            length++;
            packet_to_encode->robot.voltage = 0x08;
            length++;
            break;
    }

    packet_to_encode->total_length = length + 1;
}

void on_rx_interrupt()
{
    static bool start_of_frame = false;
    static uint8_t length = 0;
    static uint8_t read_count = 0;
    static uint8_t read_buffer[IAToMainBoard_size];
    uint8_t c;

    if (!start_of_frame) {
        serial_port.read(&c, 1);
        if (c > 0 && c <= (IAToMainBoard_size)) {
            start_of_frame = true;
            length = c;
            read_count = 0;
            // event_queue.call(printf, "Receiving : %d\n", length);
        } else if (c == 0) {
            start_of_frame = false;
            length = 0;
            read_count = 0;
            radio_packet[0] = 0;
            event_queue.call(send_protobuf_packet);
        }
    } else {
        serial_port.read(&read_buffer[read_count], 1);
        read_count++;
        if (read_count == length) {
            read_count = 0;
            start_of_frame = false;

            // event_queue.call(printf, "Parsing !\n");

            /* Try to decode protobuf response */
            ai_message = IAToMainBoard_init_zero;

            /* Create a stream that reads from the buffer. */
            pb_istream_t rx_stream = pb_istream_from_buffer(read_buffer, length);

            /* Now we are ready to decode the message. */
            bool status = pb_decode(&rx_stream, IAToMainBoard_fields, &ai_message);

            /* Check for errors... */
            if (!status) {
                event_queue.call(printf, "Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
            } else {
                radio_packet[0] = length;
                memcpy(&radio_packet[1], read_buffer, length);
                event_queue.call(printf, "CMD! Length: %d\n", length);
                event_queue.call(send_protobuf_packet);
            }
        }
    }
}

void print_radio_status()
{
    uint8_t tx_addr_device[5] = { 0 };

    radio.tx_address(tx_addr_device);
    printf("\r\nTx Address 0x");
    for (int i = 0; i < sizeof(tx_addr_device); i++) {
        printf("%.2X", tx_addr_device[i]);
    }
    printf("\r\n");
    printf("Radio status: 0x%x\n", radio.status_register());
}

int main()
{
    // Remote
    serial_port.baud(115200);
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    radio.initialize(NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, 2402);
    radio.attach_transmitting_payload(
            NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, IAToMainBoard_size + 1);
    radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, IAToMainBoard_size + 1);
    radio.set_interrupt(NRF24L01::InterruptMode::NONE);

    memset(radio_packet, 0xFF, sizeof(radio_packet));

    print_radio_status();

    event_queue.dispatch_forever();

    while (true) {
        led1 = !led1;
        if (led1) {
            printf("Alive!\n");
        }
        com_packet_t packet_to_send;
        fill_radio_packet(PACKET_MASTER, &packet_to_send);
        frame_encoder(&packet_to_send);
        printf("Sending %d bytes\n", packet_to_send.total_length);
        send_packet(packet_to_send.raw, packet_to_send.total_length);
        // send_packet(packet, sizeof(packet));
        ThisThread::sleep_for(HALF_PERIOD);

        // print_radio_status();
    }
}
