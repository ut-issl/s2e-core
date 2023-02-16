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
   * @param [in] bufSize: Buffer size
   */
  RingBuffer(int bufSize);
  /**
   * @fn ~RingBuffer
   * @brief Destructor
   */
  ~RingBuffer();

  /**
   * @fn Write
   * @brief Write data of (buffer[offset] to buffer[offset + count]) to the ring buffer's write pointer
   * @param [in] buffer: Data
   * @param [in] offset: Data offset for buffer
   * @param [in] count:  Data length for buffer
   * @return Number of bytes written
   */
  int Write(byte* buffer, int offset, int count);
  /**
   * @fn Read
   * @brief Read data at the read pointer of the ring buffer and store the data to the buffer[offset] to buffer[offset + count]
   * @param [in] buffer: Data
   * @param [in] offset: Data offset for buffer
   * @param [in] count:  Data length for buffer
   * @return Number of bytes read
   */
  int Read(byte* buffer, int offset, int count);

 private:
  const int kBufferSize;  //!< Buffer size
  byte* buf_;             //!< Buffer
  int rp_;                //!< Read pointer
  int wp_;                //!< Write pointer
};

#endif  // S2E_LIBRARY_UTILITIES_RING_BUFFER_HPP_
