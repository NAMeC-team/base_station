/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "nrf24l01.h"
#include "radio_utils.h"

namespace {
#define HALF_PERIOD 500ms
}

static DigitalOut led1(LED1);

static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };
SPI spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK);
static NRF24L01 radio(&spi, DIO1, DIO2, NC);

void blink()
{
    led1 = !led1;
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
            packet_to_encode->master.x_speed = 2000;
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

void send_packet(uint8_t *packet, size_t length)
{
    radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, length);
    radio.send_packet(packet, length);
}

void print_radio_status()
{
    printf("Radio status: 0x%x\n", radio.status_register());
}

int main()
{
    uint8_t packet[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

    radio.initialize(NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, 2402);
    radio.attach_transmitting_payload(NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, 6);
    radio.set_interrupt(NRF24L01::InterruptMode::NONE);

    print_radio_status();

    while (true) {
        led1 = !led1;
        if (led1) {
            printf("Alive!\n");
        }
        com_packet_t packet_to_send;
        fill_radio_packet(PACKET_MASTER, &packet_to_send);
        frame_encoder(&packet_to_send);
        send_packet(packet_to_send.raw, packet_to_send.total_length);
        // send_packet(packet, sizeof(packet));
        ThisThread::sleep_for(HALF_PERIOD);

        print_radio_status();
    }
}
