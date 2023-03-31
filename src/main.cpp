#include <base_wrapper.pb.h>
#include <mbed.h>
#include <nrf24l01.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <radio_command.pb.h>
#include <swo.h>

namespace {
#define HALF_PERIOD 500ms
// Radio frequency
} // namespace

#define RF_FREQUENCY_1 2402
#define RF_FREQUENCY_2 2460

EventQueue event_queue;

static DigitalOut led1(LED1);
static SPI spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF);
static NRF24L01 radio(&spi, SPI_CS_RF1, CE_RF1, IRQ_RF1);
static UnbufferedSerial serial_port(USBTX, USBRX);

static PCToBase ai_message = PCToBase_init_zero;
static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };

sixtron::SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

void send_packet(uint8_t *packet, size_t length)
{
    radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, length);
    radio.send_packet(packet, length);
}

void send_protobuf_packet(BaseCommand base_cmd)
{

    RadioCommand radio_cmd;
    radio_cmd.robot_id = base_cmd.robot_id;
    radio_cmd.normal_velocity = base_cmd.normal_velocity;
    radio_cmd.tangential_velocity = base_cmd.tangential_velocity;
    radio_cmd.angular_velocity = base_cmd.angular_velocity;
    radio_cmd.kick = base_cmd.kick;
    radio_cmd.kick_power = base_cmd.kick_power;
    radio_cmd.charge = base_cmd.charge;
    radio_cmd.dribbler = base_cmd.dribbler;
    radio_cmd.dev = false;
    event_queue.call(printf, "Robot ID %d\n", radio_cmd.robot_id);
    uint8_t tx_buffer[RadioCommand_size + 1];

    memset(tx_buffer, 0, sizeof(tx_buffer));

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream = pb_ostream_from_buffer(tx_buffer + 1, RadioCommand_size);

    bool status = pb_encode(&tx_stream, RadioCommand_fields, &radio_cmd);

    size_t message_length = tx_stream.bytes_written;

    // printf("Bytes_written: %d/%d\n", tx_stream.bytes_written, MainBoardToBrushless_size);
    // for (int i = 0; i < tx_stream.bytes_written; i++) {
    //     printf("%x ", _brushless_tx_buffer[4 + i]);
    // }
    // printf("\n");

    /* Then just check for any errors.. */
    if (!status) {
        printf("[AI] Encoding failed: %s\n", PB_GET_ERROR(&tx_stream));
        return;
    }
    tx_buffer[0] = message_length;

    radio.send_packet(tx_buffer, RadioCommand_size + 1);
}

void on_rx_interrupt()
{
    static bool start_of_frame = false;
    static uint8_t length = 0;
    static uint8_t read_count = 0;
    static uint8_t read_buffer[PCToBase_size];
    uint8_t c;

    if (!start_of_frame) {
        serial_port.read(&c, 1);
        if (c > 0 && c <= (PCToBase_size)) { // Get packet length
            start_of_frame = true;
            length = c;
            read_count = 0;
            // event_queue.call(printf, "Receiving : %d\n", length);
        } else if (c == 0) { // When length is 0 it is the default protobuf packet
            start_of_frame = false;
            length = 0;
            read_count = 0;
            // TODO: Make something
        }
    } else {
        serial_port.read(&read_buffer[read_count], 1);
        read_count++;
        if (read_count == length) {
            read_count = 0;
            start_of_frame = false;

            /* Try to decode protobuf response */
            ai_message = PCToBase_init_zero;

            /* Create a stream that reads from the buffer. */
            pb_istream_t rx_stream = pb_istream_from_buffer(read_buffer, length);

            /* Now we are ready to decode the message. */
            bool status = pb_decode(&rx_stream, PCToBase_fields, &ai_message);

            /* Check for errors... */
            if (!status) {
                event_queue.call(printf, "Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
            } else {
                for (int i = 0; i < ai_message.commands_count; i++) {
                    BaseCommand command = ai_message.commands[i];
                    event_queue.call(send_protobuf_packet, command);
                }
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

    radio.initialize(
            NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, RF_FREQUENCY_1);
    radio.attach_transmitting_payload(
            NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, RadioCommand_size + 1);
    radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, RadioCommand_size + 1);
    radio.set_interrupt(NRF24L01::InterruptMode::NONE);

    // memset(radio_packet, 0xFF, sizeof(radio_packet));

    // print_radio_status();

    event_queue.dispatch_forever();

    while (true) {
        led1 = !led1;
        ThisThread::sleep_for(HALF_PERIOD);
    }
}
