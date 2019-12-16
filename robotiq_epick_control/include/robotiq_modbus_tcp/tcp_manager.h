#ifndef ETHERCAT_MANAGER_H
#define ETHERCAT_MANAGER_H


#include <modbus/modbus.h>
#include <iostream>
#include <array>

#include <exception>
#include <ros/ros.h>

//#define FIRST_IMPLEMENTATION

namespace robotiq_modbus_tcp
{


#ifdef FIRST_IMPLEMENTATION
template<std::size_t N>
std::array<uint8_t, N> to_8bit(const std::array<uint16_t, N>&  src)
{
  std::array<uint8_t, N> out;
  for(uint8_t i = 0; i < N; i++) {
    out[i] = src[i] >> 0x8;
  }
  return out;
}
#else
template<std::size_t N>
std::array<uint8_t, 2*N> to_8bit(const std::array<uint16_t, N>& src)
{
  std::array<uint8_t, 2*N> out;
  for(uint8_t i = 0; i < N; i++) {
    out[2 * i] = src[i] >> 0x8;
    out[2 * i + 1] = src[i] & 0x00FF;
  }
  return out;
}
#endif

#ifdef FIRST_IMPLEMENTATION
template<std::size_t N>
std::array<uint16_t, N> to_16bit(const std::array<uint8_t, N>& src)
{
  std::array<uint16_t, N> out;
  for(uint8_t i=0; i<N; i++) {
    out[i] = src[i] << 0x8;
  }
  return out;
}
#else
template<std::size_t N>
std::array<uint16_t, N/2 + N%2> to_16bit(const std::array<uint8_t, N>& src)
{
  std::array<uint16_t, N/2 + N%2> out;
  for(uint8_t i=0; i<N/2; i++) {
    out[i] = (src[2*i] << 0x8) | src[2*i+1];
  }
  if(N%2 == 1) {
    out[N/2] = (src[N-1] << 0x8) | 0x0;
  }
  return out;
}
#endif


class TcpManager
{
public:

  /**
   * \brief initialise tcp manager connecting to the specified port and ip. It raise a run-time error if connections fails
   *
   * @param[in] ip address of the robotiq gripper
   * @param[in] tcp port of the robotiq gripper
  */
  TcpManager(const std::string& ip, const int port) : ip_(ip), port_(port)
  {
    mb_ = modbus_new_tcp(ip_.c_str(), port_);
    if (mb_ == NULL) {
      ROS_ERROR("Unable to allocate libmodbus context\n");
      throw std::runtime_error("Unable to allocate libmodbus context\n");
    }

    if (modbus_connect(mb_) == -1) {
      ROS_ERROR_STREAM("Connection failed: " << modbus_strerror(errno));
      modbus_free(mb_);
      throw std::runtime_error("Connection failed");
    }
  }


  ~TcpManager()
  {
    modbus_close(mb_);
    modbus_free(mb_);
  }

  /**
  * \brief writes 'src' list starting from 'channel-th' output-register and writing nb subsequent registers
  *
  * @param[in] starting register
  * @param[in] number of registers to write
  * @param[in] array of value to write at least nb * sizeof(uint8_t)
  * @return the number of written registers if successful. Otherwise it shall return -1
  *
  * This method currently makes no attempt to catch out of bounds errors. Make
  * sure that src have at least nb element properly initialised.
  */
  template<std::size_t N, uint8_t CHANNEL>
  size_t write(const std::array<uint8_t, N>& src) const
  {
    const auto src_16b = to_16bit<N>(src);
    return modbus_write_registers(mb_, CHANNEL, src_16b.size(), &src_16b[0]);
  }

  /**
  * \brief Reads nb input-registers starting from "channel-th" and store them in the dest array
  *
  * @param[in] starting register channel
  * @param[in] number of register to read
  * @param[out] output array at least nb * sizeof(uint8_t)
  * @out the number of read input registers if successful. Otherwise it shall return -1
  */
  template<std::size_t N, uint8_t CHANNEL>
  size_t readInput(std::array<uint8_t,N>& dest) const
  {
#ifdef FIRST_IMPLEMENTATION
    std::array<uint16_t, N> dest_16b;
#else
    std::array<uint16_t, N/2 + N%2> dest_16b;
#endif
    const int reg_readed = modbus_read_input_registers(mb_, CHANNEL, dest_16b.size(), &dest_16b[0]);
    dest = to_8bit(dest_16b);
    return reg_readed;
  }

  /**
  * \brief Reads nb output-registers starting from "channel-th" and store them in the dest array
  *
  * @param[in] starting register channel
  * @param[in] number of register to read
  * @param[out] output array at least nb * sizeof(uint8_t)
  * @out the number of readed input registers if successful. Otherwise it shall return -1
  */
  template<std::size_t N, uint8_t CHANNEL>
  size_t readOutput(std::array<uint8_t,N>& dest) const
  {
#ifdef FIRST_IMPLEMENTATION
    std::array<uint16_t, N> dest_16b;
#else
    std::array<uint16_t, N/2 + N%2> dest_16b;
#endif
    const int reg_readed = modbus_read_registers(mb_, CHANNEL, dest_16b.size(), &dest_16b[0]);
    dest = to_8bit(dest_16b);
    return reg_readed;
  }


private:
  const std::string ip_;
  const int port_;
  modbus_t *mb_;
};

}

#endif
