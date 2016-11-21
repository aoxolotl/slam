#ifndef _DYNAMIXEL_HEADER
#define _DYNAMIXEL_HEADER

/**
 * @file dynamixel.h
 * API for Dynamixel Servo Motor
 */
#ifdef __cplusplus
extern "C" {
#endif


///////////// device control methods ////////////////////////
/** 
 * @name dxl_initialize
 * @brief Intitializes device for communication
 * @param deviceIndex int specifying /dev/ttyUSB* number
 * @param baudnum int specifying baudrate
 * @return 
 */
int dxl_initialize(int deviceIndex, int baudnum );

/**
 * @name dxl_terminate
 * @brief closes device after end of communication
 */
void dxl_terminate();


///////////// set/get packet methods //////////////////////////
#define MAXNUM_TXPARAM		(150)
#define MAXNUM_RXPARAM		(60)

void dxl_set_txpacket_id(int id);
#define BROADCAST_ID		(254)

void dxl_set_txpacket_instruction(int instruction);
#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)

void dxl_set_txpacket_parameter(int index, int value);
void dxl_set_txpacket_length(int length);

int dxl_get_rxpacket_error(int errbit);
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

int dxl_get_rxpacket_length(void);
int dxl_get_rxpacket_parameter(int index);


// utility for value
int dxl_makeword(int lowbyte, int highbyte);
int dxl_get_lowbyte(int word);
int dxl_get_highbyte(int word);


////////// packet communication methods ///////////////////////
void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_txrx_packet(void);

int dxl_get_result(void);
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL		(2)
#define COMM_RXFAIL		(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)


//////////// high communication methods ///////////////////////
void dxl_ping(int id);
/**
 * @name dxl_read_byte
 * @brief Read byte at address
 * @param[in] id Dynamixel ID [1..254]
 * @param[in] address Address to read from
 */
int dxl_read_byte(int id, int address);

/**
 * @name dxl_write_byte
 * @brief Write byte at address
 * @param[in] id Dynamixel ID [1..254]
 * @param[in] address Address to write to
 * @param[in] value Value to be written
 */
void dxl_write_byte(int id, int address, int value);

/**
 * @name dxl_read_word
 * @brief Write word(instead of byte) at address
 * @param[in] id Dynamixel ID [1..254]
 * @param[in] address Address to read from
 */
int dxl_read_word(int id, int address);

/**
 * @name dxl_write_word
 * @brief Write byte at address
 * @param[in] id Dynamixel ID [1..254]
 * @param[in] address Address to write to
 * @param[in] value Value to be written
 */
void dxl_write_word(int id, int address, int value);

// Wrapper functions for movement etc.
/**
 * @name dxl_set_speed
 * @brief Set speed of servo in rpm
 * Only approximate value reached
 * @param[in] id Dynamixel ID
 * @param[in] rpm Speed in RPM [0...117]
 */
void dxl_set_speed(int id, float rpm);

/**
 * @name dxl_rotate_by
 * @brief Set angle of servo in degrees
 * Only approximate value reached
 * @param[in] id Dynamixel ID
 * @param[in] angle Angle in degrees [0...360)
 */
void dxl_rotate_by(int id, float angle);

#ifdef __cplusplus
}
#endif

#endif
