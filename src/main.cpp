#include <base_wrapper.pb.h>
#include <mbed.h>
#include <nrf24l01.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <radio_command.pb.h>
#include <swo.h>
#include <rf_app.h>


namespace {
#define HALF_PERIOD 500ms
#define SERIAL_BAUDRATE 115200
} // namespace

// Radio frequencies used
#define RF_FREQUENCY_1 2509 // Base station -> Robots
#define RF_FREQUENCY_2 2511 // Unused

/**
 * Event queue used to call execution of functions
 * in an interrupted context
 */
EventQueue event_queue;

static DigitalOut led1(LED1); // Debug led
static SPI spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF); // SPI handler
static NRF24L01 radio(&spi, SPI_CS_RF1, CE_RF1, IRQ_RF1); // RF module
static UnbufferedSerial serial_port(USBTX, USBRX); // Bidirectional serial comm with main computer

// (global) Current AI message received,
// updated whenever a new one is parsed
static PCToBase ai_message = PCToBase_init_zero;

// 5 bytes-long address
static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };

// Used to print out values via SWO wire. Data can be visualized using JLinkSWOViewer
sixtron::SWO swo;

// not sure what this does
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

    // remember that we send the packet like this
    // 0     8       16       32 (bits)
    // buffer = [size,         ...packet]
    // so the robot can read first byte, to see how many bytes
    // it has to read.
    // ...packet is the encoded form of a Protobuf struct message
    uint8_t tx_buffer[RadioCommand_size + 1];

    memset(tx_buffer, 0, sizeof(tx_buffer));

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream = pb_ostream_from_buffer(tx_buffer + 1, RadioCommand_size);

    bool status = pb_encode(&tx_stream, RadioCommand_fields, &radio_cmd);

    size_t message_length = tx_stream.bytes_written;

    /* Then just check for any errors.. */
    if (!status) {
        printf("[AI] Encoding failed: %s\n", PB_GET_ERROR(&tx_stream));
        return;
    }
    tx_buffer[0] = message_length;

    wait_us(400); // required to avoid on-air packet collision
    radio.send_packet(tx_buffer, RadioCommand_size + 1);
}

/**
 * Serial onRX interrupt function.
 * Called whenever a new command packet from the main computer
 * is available via USB.
 * Once the complete packet is parsed, sends them
 */
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

/**
 * Retrieve & display radio status
 */
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
    // Serial link with remote main software computer
    serial_port.baud(SERIAL_BAUDRATE);
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    // Initialize TX radio to transmit packets to robots
    radio.initialize(
            NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, RF_FREQUENCY_1);
    radio.attach_transmitting_payload(
            NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, RadioCommand_size + 1);
    radio.set_interrupt(NRF24L01::InterruptMode::NONE);
    radio.set_auto_acknowledgement(true);
    radio.set_crc(NRF24L01::CRCwidth::_8bits);
    radio.enable_dynamic_payload(true);
    radio.enable_payload_ack_mode(true);

    // Main thread executes pending events
    // -> Gets interrupted by Serial
    // -> Processes incoming packet
    // -> Queues the sending of each command for all robots

    event_queue.dispatch_forever();

    // Fallback code if EventQueue stops
    while (true) {
        led1 = !led1;
        ThisThread::sleep_for(HALF_PERIOD);
    }
}
