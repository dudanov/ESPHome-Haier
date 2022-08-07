#pragma once

#include <deque>
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart {

template<typename T> T get_buffer_data_le(const uint8_t *buffer) {
  T data = 0;
  for (size_t n = 0; n < sizeof(T); n++)
    data |= static_cast<T>(*buffer++) << (n * 8);
  return data;
}

template<typename T> T get_buffer_data_be(const uint8_t *buffer) {
  T data = 0;
  for (size_t n = sizeof(T); n;)
    data |= static_cast<T>(*buffer++) << (--n * 8);
  return data;
}

template<typename T> void set_buffer_data_le(uint8_t *buffer, const T &data) {
  for (size_t n = 0; n < sizeof(T); n++)
    *buffer++ = static_cast<uint8_t>(data >> (n * 8));
}

template<typename T> void set_buffer_data_be(uint8_t *buffer, const T &data) {
  for (size_t n = sizeof(T); n;)
    *buffer++ = static_cast<uint8_t>(data >> (--n * 8));
}

template<typename T> void append_buffer_data_le(std::vector<uint8_t> &buffer, const T &data) {
  for (size_t n = 0; n < sizeof(T); n++)
    buffer.push_back(static_cast<uint8_t>(data >> (n * 8)));
}

template<typename T> void append_buffer_data_be(std::vector<uint8_t> &buffer, const T &data) {
  for (size_t n = sizeof(T); n;)
    buffer.push_back(static_cast<uint8_t>(data >> (--n * 8)));
}

class Frame {
  friend class FrameReader;

 public:
  Frame(const uint8_t *data, size_t size) : data_(data, data + size) {}
  Frame(const std::vector<uint8_t> &data) : data_(data) {}
  Frame(std::vector<uint8_t> &&data) : data_(data) {}
  size_t get_size() const { return this->data_.size(); }
  Frame &set_bit(bool value, size_t idx, size_t nbit) {
    if (value)
      this->data_[idx] |= (1 << nbit);
    else
      this->data_[idx] &= ~(1 << nbit);
    return *this;
  }
  bool get_bit(size_t idx, size_t nbit) const { return this->data_[idx] & (1 << nbit); }
  Frame &set_data(uint8_t value, size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) {
    this->data_[idx] &= ~bitmask;
    this->data_[idx] |= (value << shift) & bitmask;
    return *this;
  }
  uint8_t get_data(size_t idx, uint8_t bitmask = 0xFF, size_t shift = 0) const {
    return (this->data_[idx] & bitmask) >> shift;
  }
  Frame &append(uint8_t data) {
    this->data_.push_back(data);
    return *this;
  }
  Frame &append(const uint8_t *data, size_t size) {
    this->data_.insert(this->data_.end(), data, data + size);
    return *this;
  }
  Frame &append(const std::vector<uint8_t> &data) {
    this->data_.insert(this->data_.end(), data.begin(), data.end());
    return *this;
  }
  Frame &operator+=(const uint8_t &data) { return this->append(data); }
  uint8_t &operator[](size_t idx) { return this->data_[idx]; }
  const uint8_t &operator[](size_t idx) const { return this->data_[idx]; }
  template<typename T> T get_le(size_t idx) const { return get_buffer_data_le(&this->data_[idx]); }
  template<typename T> T get_be(size_t idx) const { return get_buffer_data_be(&this->data_[idx]); }
  template<typename T> void set_le(const T &data, size_t idx) { set_buffer_data_le(&this->data_[idx], data); }
  template<typename T> void set_be(const T &data, size_t idx) { set_buffer_data_be(&this->data_[idx], data); }
  template<typename T> void append_le(const T &data) { append_buffer_data_le(this->data_, data); }
  template<typename T> void append_be(const T &data) { append_buffer_data_be(this->data_, data); }

 protected:
  std::vector<uint8_t> data_;
};

class FrameReader : public Frame, public Component {
 public:
  using on_receive_cb = std::function<bool(uint8_t, std::vector<uint8_t> &)>;
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
  std::deque<Frame *> write_queue_;
  UARTComponent *uart_{};
  on_receive_cb on_receive_cb_;
  on_frame_cb on_frame_cb_;
};

class Protocol {
  virtual bool on_data(Frame &frame, uint8_t data) = 0;
  virtual void on_frame(FrameReader &reader) = 0;
};

}  // namespace uart
}  // namespace esphome
