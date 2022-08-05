#pragma once

#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart {

class Frame {
  friend class FrameReader;

 public:
 protected:
  std::vector<uint8_t> data_;
  bool get_bit_(size_t idx, size_t nbit) const { return this->data_[idx] & (1 << nbit); }
  void set_bit_(bool value, size_t idx, size_t nbit) {
    if (value)
      this->data_[idx] |= (1 << nbit);
    else
      this->data_[idx] &= ~(1 << nbit);
  }
  uint8_t get_data_(size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) const {
    return (this->data_[idx] & bitmask) >> shift;
  }
  /// Set value to data buffer on specified index, bitmask
  void set_data_(uint8_t value, size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) {
    this->data_[idx] &= ~bitmask;
    this->data_[idx] |= (value << shift) & bitmask;
  }
};

class FrameReader : public Frame, public Component {
 public:
  using receive_cb = std::function<bool(uint8_t, std::vector<uint8_t> &)>;
  using callback = std::function<void(FrameReader &)>;
  // Set UART component
  void set_uart_parent(UARTComponent *parent) { this->uart_ = parent; }
  // Set callback function
  void on_frame(callback cb) { this->cb_ = cb; }
  // Set callback function
  void on_receive(receive_cb cb) { this->receive_cb_ = cb; }
  // Write this frame to UART
  void write() { this->write(*this); }
  // Write other frame to UART
  void write(const Frame &frame) { this->uart_->write_array(frame.data_); }
  // Read frame. Call callback function if valid frame was received.
  void loop() override;

 protected:
  UARTComponent *uart_{};
  receive_cb receive_cb_{};
  callback cb_{};
};

}  // namespace uart
}  // namespace esphome
