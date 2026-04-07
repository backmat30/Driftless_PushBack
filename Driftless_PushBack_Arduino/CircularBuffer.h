#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include <array>

/**
* Compile-time based ring buffer for bytes
*/
template<size_t N>
class CircularBuffer {
  private:
    std::array<char, N> buffer{};

    size_t head{};

    size_t tail{};

    size_t count{};

  public:
    /**
    * Constructs a new CircularBuffer object
    */
    CircularBuffer() {}

/**
* Determines if the circular buffer is full
* @return True if full, false if there is at least 1 free space in the buffer
*/
    bool isFull() {
      return count == N;
    }

/**
* Determines if the circular buffer is empty
* @return True if empty, false if there is at least 1 byte in the buffer
*/
    bool isEmpty() {
      return count == 0;
    }

/**
* Determines the number of bytes stored in the buffer
* @return The number of bytes in the buffer
*/
    size_t size() {
      return count;
    }

/**
* Reads the next available byte and stores it in the provided space
* @param byte The space to store the next byte
* @return True if successful, false otherwise
*/
    bool readNext(char& byte) {
      if (isEmpty()) {
        return false;
      }

      byte = buffer[tail++];
      tail %= N;
      count--;

      return true;
    }

/**
* Writes a byte to the circular buffer
* @param byte The byte to write to the buffer
* @return True if successful, false otherwise
*/
    bool write(char byte) {
      if(isFull()) {
        return false;
      }

      buffer[head++] = byte;
      head %= N;
      count++;

      return true;
    }
}

#endif