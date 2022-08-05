#include "Frame.h"

namespace esphome {
namespace uart {

void FrameReader::loop() {
  while (this->uart_->available() > 0) {
    uint8_t x;
    this->uart_->read_byte(&x);
    if (this->receive_cb_(x, this->data_))
      this->cb_(*this);
  }
}

}  // namespace uart
}  // namespace esphome
