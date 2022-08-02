#include "Haier.h"

void Frame::update_crc() {
  const size_t size = this->buf_.size();
  if (size <= OFFSET_LENGTH)
    return;
  const size_t idx = this->get_length_() + 3;
  if (idx > size)
    return;
  if (idx < size)
    this->buf_.erase(this->buf_.begin() + idx, this->buf_.end());
  const auto crc16 = this->calc_crc16_();
  this->buf_.push_back(this->calc_crc8_());
  this->buf_.push_back(crc16 / 256);
  this->buf_.push_back(crc16 % 256);
}

uint8_t Frame::calc_crc8_() const {
  uint8_t crc = 0;
  if (this->buf_.size() > OFFSET_LENGTH) {
    auto it = this->buf_.begin() + OFFSET_LENGTH;
    const auto end = it + this->get_length_();
    while (it != end)
      crc += *it++;
  }
  return crc;
}

uint16_t Frame::calc_crc16_() const {
  uint16_t crc = 0;
  if (this->buf_.size() > OFFSET_LENGTH) {
    auto it = this->buf_.begin() + OFFSET_LENGTH;
    const auto end = it + this->get_length_();
    while (it != end) {
      crc ^= static_cast<uint16_t>(*it++);
      for (size_t n = 0; n < 8; n++)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0x0000);
    }
  }
  return crc;
}

void FrameReader::read() {
  while (this->uart_->available() > 0) {
    uint8_t data;
    const auto idx = this->buf_.size();
    this->uart_->read_byte(&data);
    if (idx < OFFSET_LENGTH && data != 255) {
      this->buf_.clear();
      continue;
    }
    this->buf_.push_back(data);
    if (idx >= 5 && (idx - 5) == this->get_length_()) {
      if (this->is_valid_())
        this->cb_(*this);
      this->buf_.clear();
    }
  }
}

bool FrameReader::is_valid_() const {
  const auto p = this->buf_.end();
  const uint16_t crc16 = 256 * p[-2] + p[-1];
  const uint8_t crc8 = p[-3];
  return this->calc_crc8_() == crc8 && this->calc_crc16_() == crc16;
}
