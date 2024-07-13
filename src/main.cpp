#include <base_wrapper.pb.h>
#include <mbed.h>
#include <nrf24l01.h>
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <radio_command.pb.h>
#include <radio_feedback.pb.h>
#include <swo.h>
#include <rf_app.h>

#define MAX_ROBOT_ID 7

namespace {
#define HALF_PERIOD 500ms
// Radio frequency
} // namespace

#define RF_FREQUENCY_1 2508
#define RF_FREQUENCY_2 2510

// -- Global variables

EventQueue event_queue;

/**
 * Global message response buffer
 */
static BaseToPC base_response = BaseToPC_init_zero;
/**
 * Number of robot commands to send, in a single iteration
 * which is also the number of feedbacks to potentially receive
 */
static size_t num_robot_commands = 0;

static bool feedbacks_sent = true;

/**
 * Timeout to trigger the function to send feedbacks to the PC
 * if it takes too much time to receive all the feedbacks
 */
static Timeout feedbacks_timeout;

// given by nrf24l01+ datasheet
#define ANTENNA_SWAP_TIME 130us

// timings based on packet size
// must be changed if packets are changed
#define BASECMD_SEND_TIME 140us
#define ROBOT_SEND_TIME 132us

/**
 * Timeout time for a single robot
 * Must multiply by the number of robots to get total feedback count
 * last value is a tolerance margin
 */
#define FDB_TIMEOUT_TIME (BASECMD_SEND_TIME + ANTENNA_SWAP_TIME + ROBOT_SEND_TIME + 500000us)

static DigitalOut led1(LED1);

// SPI bus handler
static SPI spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF);

/**
 * Address to send commands to, and to receive feedback on
 * Both antennas send and listen on the same address, but on
 * different frequencies
 */
static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };

/**
 * Transmission radio used to send orders to the robots
 * on RF_FREQUENCY_1
 * Uses the antenna on COM1 port
 */
static NRF24L01 tx_radio(&spi, SPI_CS_RF1, CE_RF1, IRQ_RF1);

/**
 * Reception radio used to listen to the robots sending
 * their feedback, on RF_FREQUENCY_2
 * Uses the antenna on COM2 port
 * Should not be handled, use the wrapper instead
 */
static NRF24L01 rx_radio(&spi, SPI_CS_RF2, CE_RF2, IRQ_RF2);

static RF_app w_rx_radio(
        &rx_radio,
        RF_app::RFAppInterrupt::on_RX,
        RF_app::RFAppMode::RX,
        RF_FREQUENCY_2,
        com_addr1_to_listen,
        RadioFeedback_size
);

/**
 * Serial communication with the computer having the AI software
 */
static UnbufferedSerial serial_port(USBTX, USBRX);

/**
 * Global message edited when receiving a message from the serial link
 */
static PCToBase ai_message = PCToBase_init_zero;

sixtron::SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
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
//    event_queue.call(printf, "Robot ID %d\n", radio_cmd.robot_id);
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

    tx_radio.send_packet(tx_buffer, RadioCommand_size + 1);
}

BaseFeedback convert_packet(RadioFeedback *to_convert) {
    BaseFeedback fb = BaseFeedback_init_zero;
    fb.robot_id = to_convert->robot_id;
    fb.ir = to_convert->ir;
    fb.motor_1_speed = to_convert->motor_1_speed;
    fb.motor_2_speed = to_convert->motor_2_speed;
    fb.motor_3_speed = to_convert->motor_3_speed;
    fb.motor_4_speed = to_convert->motor_4_speed;
    fb.voltage = to_convert->voltage;
    return fb;
}

void send_feedbacks_to_pc()
{
    if (feedbacks_sent)
        return;

    serial_port.rewind();

    if (base_response.feedbacks_count == 0) {
        uint8_t buffer[1] = { 0 };
        serial_port.write(buffer, sizeof(uint8_t));
        serial_port.sync();
        led1 = false;
        return;
    }

    uint8_t tx_buffer[BaseToPC_size + 1];

    pb_ostream_t tx_stream = pb_ostream_from_buffer(tx_buffer + 1, PCToBase_size);

    bool status = pb_encode(&tx_stream, BaseToPC_fields, &base_response);

    // `base_response` already gets filled before this function is called

    if (!status)
        return;

    tx_buffer[0] = tx_stream.bytes_written;
    serial_port.write(tx_buffer, tx_stream.bytes_written + 1);
    serial_port.sync();
    base_response = BaseToPC_init_zero;
    base_response.feedbacks_count = 0;
    num_robot_commands = 0;
    feedbacks_sent = true;
    led1 = false;
}

void on_rx_interrupt()
{
    led1 = true;
    static bool start_of_frame = false;
    static uint8_t length = 0;
    static uint8_t read_count = 0;
    static uint8_t read_buffer[PCToBase_size];
    uint8_t c;

    if (!start_of_frame) {
        serial_port.rewind();
        //TODO: if we exceed 6 robots, the size of the packet exceeds 8 bits,
        // thus the length will be stored on 2 bytes, making this algorithm invalid
        // not a problem for DivB, but it will be a problem for DivA
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
//                event_queue.call(printf, "Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
            } else {
                if (ai_message.commands_count == 0)
                    return;

                feedbacks_sent = false;

                num_robot_commands = ai_message.commands_count;
                for (int i = 0; i < ai_message.commands_count; i++) {
                    BaseCommand command = ai_message.commands[i];
                    event_queue.call(send_protobuf_packet, command);
                }
                // force sending after a certain timeout if all
                // feedbacks could not be received
                feedbacks_timeout.attach(send_feedbacks_to_pc, num_robot_commands * FDB_TIMEOUT_TIME);
            }
        }
    }
}

void on_rx_rf_interrupt(uint8_t *packet, size_t data_length)
{
    RadioFeedback feedback = RadioFeedback_init_zero;
    uint8_t length = packet[0];
    if (length == 0)
        return;

    pb_istream_t rx_stream = pb_istream_from_buffer(&packet[1], length);
    bool status = pb_decode(&rx_stream, RadioFeedback_fields, &feedback);

    if (!status)
        return;

    // at this point, the new message received is valid
    base_response.feedbacks[base_response.feedbacks_count] = convert_packet(&feedback);
    base_response.feedbacks_count += 1;

    // when all responses are received, send feedback to pc
    if (base_response.feedbacks_count >= num_robot_commands) {
        feedbacks_timeout.detach();
        event_queue.call(send_feedbacks_to_pc);
    }
}

int main()
{
    // Remote
    serial_port.baud(460800);
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    // setup robot orders transmission radio
    tx_radio.initialize(
            NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, RF_FREQUENCY_1);
    tx_radio.attach_transmitting_payload(
            NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, RadioCommand_size + 1);
    tx_radio.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, RadioCommand_size + 1);
    tx_radio.set_interrupt(NRF24L01::InterruptMode::NONE);

    // setup robot feedback reception radio
    w_rx_radio.attach_rx_callback(&on_rx_rf_interrupt);
    w_rx_radio.run();

    event_queue.dispatch_forever();

    while (true) {}
}
