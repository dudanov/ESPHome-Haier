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

// Простой статический вектор
template<typename _Tp, std::size_t _size>
class StaticVector {
 public:
  typedef _Tp value_type;
  typedef _Tp* iterator;
  typedef const _Tp* const_iterator;
  typedef _Tp* pointer;
  typedef const _Tp* const_pointer;
  typedef _Tp& reference;
  typedef const _Tp& const_reference;
  StaticVector() : m_end(m_buf.begin()) {}
  StaticVector(const StaticVector &s) { this->assign(s.cbegin(), s.cend()); }
  StaticVector(const_pointer data, std::size_t size) { this->assign(data, data + size); }
  StaticVector(std::initializer_list<_Tp> list) { this->assign(list.begin(), list.end()); }
  void assign(const_iterator begin, const_iterator end) { this->m_end = std::copy(begin, end, this->begin()); }
  void fill(iterator pos, std::size_t n, const_reference val) { this->m_end = std::fill_n(pos, n, val); }
  void fill(std::size_t idx, std::size_t n, const_reference val) { this->fill(this->begin() + idx, n, val); }
  void fill(std::size_t n, const_reference val) { this->fill(this->begin(), n, val); }
  iterator begin() { return this->m_buf.begin(); }
  iterator end() { return this->m_end; }
  const_iterator begin() const { return this->m_buf.begin(); }
  const_iterator end() const { return this->m_end; }
  const_iterator cbegin() const { return this->m_buf.begin(); }
  const_iterator cend() const { return this->m_end; }
  void clear() { this->m_end = this->m_buf.begin(); }
  void trim(std::size_t idx) { this->m_end = &this->m_buf[idx]; }
  // Удаляет первый равный элемент
  void remove_first(const_reference val) {
    for (auto it = this->begin(); it != this->end(); ++it) {
      if (*it == val) {
        erase(it);
        return;
      }
    }
  }
  // Удаляет все равные элементы
  void remove(const_reference val) {
    for (auto it = this->begin(); it != this->end(); ) {
      if (*it == val)
        it = erase(it);
      else
        ++it;
    }
  }
  // Эффективно удаляет элемент, заменяя на крайний если таковым не является.
  // Размер уменьшается на единицу. Порядок элементов нарушается!
  // Для совместимости возвращает итератор на следующий элемент.
  iterator erase(iterator it) {
    if (!this->last(it))
      *it = std::move(this->back());
    this->pop_back();
    return it;
  }
  bool last(std::size_t idx) const { return (idx + 1) == this->size(); }
  bool last(const_iterator it) const { return std::next(it) == this->cend(); }
  bool valid(std::size_t idx) const { return idx < this->size(); }
  bool empty() const { return this->cend() == this->cbegin(); }
  void push_back(const_reference data) { *this->m_end++ = data; }
  void pop_back() { --this->m_end; }
  const_reference front() const { return *this->cbegin(); }
  const_reference back() const { return *(this->cend() - 1); }
  const_pointer data() const { return this->m_buf.data(); }
  std::size_t size() const { return std::distance(this->cbegin(), this->cend()); }
  std::size_t capacity() const { return _size; }
  std::size_t max_size() const { return _size; }
  std::size_t free_size() const { return this->max_size() - this->size(); }
  reference at(size_t idx) { return *(this->begin() + idx); }
  const_reference at(size_t idx) const { return *(this->cbegin() + idx); }
  reference operator[](size_t idx) { return this->m_buf[idx]; }
  const_reference operator[](size_t idx) const { return this->m_buf[idx]; }
 private:
  iterator m_end;
  std::array<_Tp, _size> m_buf;
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
    this->data_.insert(this->data_.end(), data);
    return *this;
  }
  Frame &operator+=(const uint8_t &data) { return this->append(data); }
  uint8_t &operator[](size_t idx) { return this->data_[idx]; }
  const uint8_t &operator[](size_t idx) const { return this->data_[idx]; }
  template<typename T> T get_le(size_t idx) const { return get_buffer_data_le(&this->data_[idx]); }
  template<typename T> T get_be(size_t idx) const { return get_buffer_data_be(&this->data_[idx]); }
  template<typename T> Frame &set_le(const T &data, size_t idx) {
    set_buffer_data_le(&this->data_[idx], data);
    return *this;
  }
  template<typename T> Frame &set_be(const T &data, size_t idx) {
    set_buffer_data_be(&this->data_[idx], data);
    return *this;
  }
  template<typename T> Frame &append_le(const T &data) {
    append_buffer_data_le(this->data_, data);
    return *this;
  }
  template<typename T> Frame &append_be(const T &data) {
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
