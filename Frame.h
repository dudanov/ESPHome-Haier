#pragma once

#include <deque>
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart {

class Frame {
  friend class FrameReader;

 public:
  typedef std::vector<uint8_t> buffer_t;

 protected:
  buffer_t data_;
  bool get_bit(size_t idx, size_t nbit) const { return this->data_[idx] & (1 << nbit); }
  void set_bit(bool value, size_t idx, size_t nbit) {
    if (value)
      this->data_[idx] |= (1 << nbit);
    else
      this->data_[idx] &= ~(1 << nbit);
  }
  uint8_t get_data(size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) const {
    return (this->data_[idx] & bitmask) >> shift;
  }
  void set_data(uint8_t value, size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) {
    this->data_[idx] &= ~bitmask;
    this->data_[idx] |= (value << shift) & bitmask;
  }
};

class FrameReader : public Frame, public Component {
 public:
  using on_receive_cb = std::function<bool(uint8_t, buffer_t &)>;
  using on_frame_cb = std::function<void(FrameReader &)>;
  // Set UART component
  void set_uart_parent(UARTComponent *parent) { this->uart_ = parent; }
  // Set callback function
  void on_frame(on_frame_cb fn) { this->on_frame_cb_ = fn; }
  // Set callback function
  void on_receive(on_receive_cb fn) { this->on_receive_cb_ = fn; }
  // Write this frame to UART
  void write() { this->write(*this); }
  // Write other frame to UART
  void write(const Frame &frame) { this->uart_->write_array(frame.data_); }
  // Read frame. Call callback function if valid frame was received.
  void loop() override;

 protected:
  UARTComponent *uart_{};
  
  on_receive_cb on_receive_cb_;
  on_frame_cb on_frame_cb_;
};

}  // namespace uart
}  // namespace esphome
