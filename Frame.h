#pragma once

#include <deque>
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart {

template<typename T> T get_buffer_data_le(const uint8_t *buffer) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  T data = 0;
  for (size_t n = 0; n < sizeof(T); n++)
    data |= static_cast<T>(*buffer++) << (n * 8);
  return data;
}

template<typename T> T get_buffer_data_be(const uint8_t *buffer) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  T data = 0;
  for (size_t n = sizeof(T); n;)
    data |= static_cast<T>(*buffer++) << (--n * 8);
  return data;
}

template<typename T> void set_buffer_data_le(uint8_t *buffer, const T &data) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  for (size_t n = 0; n < sizeof(T); n++)
    *buffer++ = static_cast<uint8_t>(data >> (n * 8));
}

template<typename T> void set_buffer_data_be(uint8_t *buffer, const T &data) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  for (size_t n = sizeof(T); n;)
    *buffer++ = static_cast<uint8_t>(data >> (--n * 8));
}

template<typename T> void append_buffer_data_le(std::vector<uint8_t> &buffer, const T &data) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  for (size_t n = 0; n < sizeof(T); n++)
    buffer.push_back(static_cast<uint8_t>(data >> (n * 8)));
}

template<typename T> void append_buffer_data_be(std::vector<uint8_t> &buffer, const T &data) {
  static_assert(std::is_integral<T>::value, "T must be an integer.");
  for (size_t n = sizeof(T); n;)
    buffer.push_back(static_cast<uint8_t>(data >> (--n * 8)));
}

template<typename T, size_t size_> class static_vector {
 public:
  typedef T value_type;
  typedef value_type *iterator;
  typedef const value_type *const_iterator;
  typedef value_type *pointer;
  typedef const value_type *const_pointer;
  typedef value_type &reference;
  typedef const value_type &const_reference;
  static_vector() : end_(this->buf_.begin()) {}
  static_vector(const static_vector &s) { this->assign(s.begin(), s.end()); }
  static_vector(const_pointer data, size_t size) { this->assign(data, data + size); }
  static_vector(std::initializer_list<T> list) { this->assign(list.begin(), list.end()); }
  constexpr iterator begin() { return this->buf_.begin(); }
  constexpr const_iterator begin() const { return this->buf_.begin(); }
  constexpr const_iterator cbegin() const { return this->buf_.begin(); }
  iterator end() { return this->end_; }
  const_iterator end() const { return this->end_; }
  const_iterator cend() const { return this->end_; }
  constexpr pointer data() { return this->buf_.data(); }
  constexpr const_pointer data() const { return this->buf_.data(); }
  size_t size() const { return std::distance(this->begin(), this->end()); }
  constexpr size_t capacity() const { return size_; }
  constexpr size_t max_size() const { return size_; }
  size_t free_size() const { return this->max_size() - this->size(); }
  bool empty() const { return this->end() == this->begin(); }
  void clear() { this->end_ = this->begin(); }
  void push_back(const_reference data) { *this->end_++ = data; }
  void pop_back() { --this->end_; }
  constexpr reference front() { return this->buf_.front(); }
  constexpr const_reference front() const { return this->buf_.front(); }
  reference back() { return *std::prev(this->end_); }
  const_reference back() const { return *std::prev(this->end_); }
  reference at(size_t idx) { return this->buf_.at(idx); }
  const_reference at(size_t idx) const { return this->buf_.at(idx); }
  reference operator[](size_t idx) { return this->buf_[idx]; }
  const_reference operator[](size_t idx) const { return this->buf_[idx]; }

  void assign(const_iterator begin, const_iterator end) { this->end_ = std::copy(begin, end, this->begin()); }
  void fill(iterator pos, ssize_t n, const_reference value) { this->end_ = std::fill_n(pos, n, value); }
  void fill(size_t idx, ssize_t n, const_reference value) { this->fill(this->begin() + idx, n, value); }
  void fill(size_t n, const_reference value) { this->fill(this->begin(), n, value); }
  void resize(size_t new_size) { this->end_ = this->begin() + new_size; }
  void resize(size_t new_size, const_reference value) {
    this->fill(this->end(), new_size - this->size(), value);
    this->resize(new_size);
  }
  void remove_first(const_reference value) {
    for (auto it = this->begin(); it != this->end(); ++it) {
      if (*it == value) {
        this->erase(it);
        return;
      }
    }
  }
  void remove(const_reference value) {
    for (auto it = this->begin(); it != this->end();) {
      if (*it == value)
        it = this->erase(it);
      else
        ++it;
    }
  }
  iterator erase(iterator it) {
    if (!this->last(it))
      *it = std::move(this->back());
    this->pop_back();
    return it;
  }
  bool last(size_t idx) const { return (idx + 1) == this->size(); }
  bool last(const_iterator it) const { return std::next(it) == this->end(); }
  bool valid(size_t idx) const { return idx < this->size(); }

 private:
  iterator end_;
  std::array<value_type, size_> buf_;
};

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
  Frame &append(std::initializer_list<uint8_t> data) {
    this->data_.insert(this->data_.end(), data.begin(), data.end());
    return *this;
  }
  Frame &operator+=(const uint8_t &data) { return this->append(data); }
  uint8_t &operator[](size_t idx) { return this->data_[idx]; }
  const uint8_t &operator[](size_t idx) const { return this->data_[idx]; }
  template<typename T> T get_le(size_t idx) const {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    return get_buffer_data_le(&this->data_[idx]);
  }
  template<typename T> T get_be(size_t idx) const {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    return get_buffer_data_be(&this->data_[idx]);
  }
  template<typename T> Frame &set_le(const T &data, size_t idx) {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    set_buffer_data_le(&this->data_[idx], data);
    return *this;
  }
  template<typename T> Frame &set_be(const T &data, size_t idx) {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    set_buffer_data_be(&this->data_[idx], data);
    return *this;
  }
  template<typename T> Frame &append_le(const T &data) {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    append_buffer_data_le(this->data_, data);
    return *this;
  }
  template<typename T> Frame &append_be(const T &data) {
    static_assert(std::is_integral<T>::value, "T must be an integer.");
    append_buffer_data_be(this->data_, data);
    return *this;
  }

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
  virtual bool on_data(Frame &frame, const uint8_t *data, size_t size) = 0;
  virtual void on_frame(FrameReader &reader) = 0;
};

}  // namespace uart
}  // namespace esphome
