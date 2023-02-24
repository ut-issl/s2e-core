/**
 * @file ring_buffer.hpp
 * @brief Class to emulate ring buffer
 */

#ifndef S2E_LIBRARY_UTILITIES_RING_BUFFER_HPP_
#define S2E_LIBRARY_UTILITIES_RING_BUFFER_HPP_

typedef unsigned char byte;

/**
 * @class RingBuffer
 * @brief Class to emulate ring buffer
 */
class RingBuffer {
 public:
  /**
   * @fn RingBuffer
   * @brief Constructor
   * @param [in] buffer_size: Buffer size
   */
  RingBuffer(int buffer_size);
  /**
   * @fn ~RingBuffer
   * @brief Destructor
   */
  ~RingBuffer();

  /**
   * @fn Write
   * @brief Write data of (buffer[offset] to buffer[offset + data_length]) to the ring buffer's write pointer
   * @param [in] buffer: Data
   * @param [in] offset: Data offset for buffer
   * @param [in] data_length:  Data length for buffer
   * @return Number of bytes written
   */
  int Write(byte* buffer, int offset, int data_length);
  /**
   * @fn Read
   * @brief Read data at the read pointer of the ring buffer and store the data to the buffer[offset] to buffer[offset + data_length]
   * @param [in] buffer: Data
   * @param [in] offset: Data offset for buffer
   * @param [in] data_length:  Data length for buffer
   * @return Number of bytes read
   */
  int Read(byte* buffer, int offset, int data_length);

 private:
  int buffer_size_;    //!< Buffer size
  byte* buffer_;       //!< Buffer
  int read_pointer_;   //!< Read pointer
  int write_pointer_;  //!< Write pointer
};

#endif  // S2E_LIBRARY_UTILITIES_RING_BUFFER_HPP_
