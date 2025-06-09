/*
 * @file c2a_communication.h
 * @brief C2A communication functions
 */

#ifndef C2A_COMMUNICATION_H_
#define C2A_COMMUNICATION_H_

// If the character encoding of C2A is UTF-8, the following functions are not necessary,
// and users can directory use SendFromObc_C2A and ReceivedByObc_C2A UART
// TODO: Delete these functions since C2A is changed to use UTF-8

// C2A communication functions
int OBC_C2A_SendFromObc(int port_id, unsigned char* buffer, int offset, int length);
int OBC_C2A_ReceivedByObc(int port_id, unsigned char* buffer, int offset, int length);

// I2C
int OBC_C2A_I2cWriteCommand(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length);
int OBC_C2A_I2cWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length);
int OBC_C2A_I2cReadRegister(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length);

// GPIO
int OBC_C2A_GpioWrite(int port_id, const bool is_high);
bool OBC_C2A_GpioRead(int port_id);  // return false when the port_id is not used

#endif  // C2A_COMMUNICATION_H_
