#include "amis.h"
#include "aes.h"
#include "esphome/core/log.h"
#include <sstream>
#include <iomanip>

#define CHR2BIN(c) (c - (c >= 'A' ? 55 : 48))

#define OFFS_DIF 19
#define timeout 10000

namespace esphome
{
  namespace amis
  {

    void amis::AMISComponent::setup()
    {
      ESP_LOGI(TAG, "AMIS smart meter component v%s started", AMIS_VERSION);
    }

    void amis::AMISComponent::hex2bin(const std::string s, uint8_t *buf)
    {
      unsigned len = s.length();
      if (len != 32)
        return;

      for (unsigned i = 0; i < len; ++i)
      {
        buf[i] = CHR2BIN(s.c_str()[i * 2]) << 4 | CHR2BIN(s.c_str()[i * 2 + 1]);
      }
    }

    void amis::AMISComponent::set_power_grid_key(const std::string &power_grid_key)
    {
      this->hex2bin(power_grid_key, this->key);
    }

    uint8_t amis::AMISComponent::dif2len(uint8_t dif)
    {
      switch (dif & 0x0F)
      {
      case 0x0:
        return 0;
      case 0x1:
        return 1;
      case 0x2:
        return 2;
      case 0x3:
        return 3;
      case 0x4:
        return 4;
      case 0x5:
        return 4;
      case 0x6:
        return 6;
      case 0x7:
        return 8;
      case 0x8:
        return 0;
      case 0x9:
        return 1;
      case 0xA:
        return 2;
      case 0xB:
        return 3;
      case 0xC:
        return 4;
      case 0xD:
        // variable data length,
        // data length stored in data field
        return 0;
      case 0xE:
        return 6;
      case 0xF:
        return 8;

      default: // never reached
        return 0x00;
      }
    }

    void amis::AMISComponent::amis_decode()
    {
      char cs = 0;
      int i;
      uint32_t temp;
      struct tm t;
      uint8_t dif;
      uint8_t vif;
      uint8_t dife;
      uint8_t vife;
      uint8_t data_len;

      if (this->receive_buffer_index < 78)
      {
        ESP_LOGD(TAG, "received incomplete frame");
        this->reset_telegram();
      }

      for (i = 4; i < receive_buffer_index - 2; i++)
        cs += this->receive_buffer[i];

      if (cs == this->receive_buffer[this->receive_buffer_index - 2])
      {
        ESP_LOGD(TAG, "checksum correct, decrypting");

        this->iv[0] = this->receive_buffer[11];
        this->iv[1] = this->receive_buffer[12];
        this->iv[2] = this->receive_buffer[7];
        this->iv[3] = this->receive_buffer[8];
        this->iv[4] = this->receive_buffer[9];
        this->iv[5] = this->receive_buffer[10];
        this->iv[6] = this->receive_buffer[13];
        this->iv[7] = this->receive_buffer[14];
        for (int i = 8; i < 16; i++)
          this->iv[i] = this->receive_buffer[15];

        AES128_CBC_decrypt_buffer(this->decode_buffer, this->receive_buffer + OFFS_DIF + 0, 16, this->key, this->iv);
        AES128_CBC_decrypt_buffer(this->decode_buffer + 16, this->receive_buffer + OFFS_DIF + 16, 16, 0, 0);
        AES128_CBC_decrypt_buffer(this->decode_buffer + 32, this->receive_buffer + OFFS_DIF + 32, 16, 0, 0);
        AES128_CBC_decrypt_buffer(this->decode_buffer + 48, this->receive_buffer + OFFS_DIF + 48, 16, 0, 0);
        AES128_CBC_decrypt_buffer(this->decode_buffer + 64, this->receive_buffer + OFFS_DIF + 64, 16, 0, 0);

        if (this->decode_buffer[0] != 0x2f || this->decode_buffer[1] != 0x2f)
        {
          ESP_LOGD(TAG, "decryption sanity check failed.");
          this->reset_telegram();
        }

        // https://github.com/volkszaehler/vzlogger/blob/master/src/protocols/MeterOMS.cpp
        // line 591

        i = 2;
        // 80 is the maximum size of data that we decrypt
        while (i < 80)
        {
          dif = this->decode_buffer[i];
          if (dif == 0x0f or dif == 0x1f)
          {
            ESP_LOGE(TAG, "Variable length not supported.");
            this->reset_telegram();
          }
          dife = 0;
          vife = 0;
          data_len = this->dif2len(dif);

          if (dif & 0x80)
          {
            while (this->decode_buffer[i] & 0x80)
            {
              dife = this->decode_buffer[i + 1];
              i++;
            }
          }

          i++;

          vif = this->decode_buffer[i];
          if (vif == 0x7c)
          {
            ESP_LOGE(TAG, "Variable length vif not supported.");
            this->reset_telegram();
          }

          if (vif & 0x80)
          {
            while (this->decode_buffer[i] & 0x80)
            {
              vife = this->decode_buffer[i + 1];
              i++;
            }
          }
          if ((dif & 0x0f) == 0x0d)
          {
            ESP_LOGE(TAG, "Variable length data not supported.");
            this->reset_telegram();
          }

          i++;

          switch (vif)
          {
          case 0x6d:
            t.tm_sec = this->decode_buffer[i] & 0x3f;
            t.tm_min = this->decode_buffer[i + 1] & 0x3f;
            t.tm_hour = this->decode_buffer[i + 2] & 0x1f;
            t.tm_mday = this->decode_buffer[i + 3] & 0x1f;
            t.tm_mon = this->decode_buffer[i + 4] & 0xf;
            if (t.tm_mon > 0)
              t.tm_mon -= 1;
            t.tm_year = 100 + (((this->decode_buffer[i + 3] & 0xe0) >> 5) | ((this->decode_buffer[i + 4] & 0xf0) >> 1));
            t.tm_isdst = ((this->decode_buffer[i] & 0x40) == 0x40) ? 1 : 0;

            if ((this->decode_buffer[i + 1] & 0x80) == 0x80)
            {
              ESP_LOGD(TAG, "time invalid");
              this->reset_telegram();
            }
            else
            {
              ESP_LOGD(TAG, "time=%.2d-%.2d-%.2d %.2d:%.2d:%.2d",
                       1900 + t.tm_year, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
              ESP_LOGD(TAG, "timestamp=%ld", mktime(&t));
            }
            if (this->timestamp_sensor)
              this->timestamp_sensor->publish_state(mktime(&t));
            break;
          case 0x03:
            if (dif == 0x04)
            {
              // 1.8.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "1.8.0: %d", temp);
              if (this->energy_a_positive_sensor)
                this->energy_a_positive_sensor->publish_state(temp);
            }
            break;
          case 0x83:
            if (dif == 0x04 && vife == 0x3c)
            {
              // 2.8.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "2.8.0: %d", temp);
              if (this->energy_a_negative_sensor)
                this->energy_a_negative_sensor->publish_state(temp);
            }
            break;
          case 0xfb:
            if (dif == 0x84 && dife == 0x10 && vife == 0x73)
            {
              // 3.8.1
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "3.8.1: %d", temp);
              if (this->reactive_energy_a_positive_sensor)
                this->reactive_energy_a_positive_sensor->publish_state(temp);
            }
            if (dif == 0x84 && dife == 0x10 && vife == 0x3c)
            {
              // 4.8.1
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "4.8.1: %d", temp);
              if (this->reactive_energy_a_negative_sensor)
                this->reactive_energy_a_negative_sensor->publish_state(temp);
            }
            if (dif == 0x04 && dife == 0x00 && vife == 0x14)
            {
              // 3.7.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "3.7.0: %d", temp);
              if (this->reactive_instantaneous_power_a_positive_sensor)
                this->reactive_instantaneous_power_a_positive_sensor->publish_state(temp);
            }
            if (dif == 0x04 && dife == 0x00 && vife == 0x3c)
            {
              // 4.7.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "4.7.0: %d", temp);
              if (this->reactive_instantaneous_power_a_negative_sensor)
                this->reactive_instantaneous_power_a_negative_sensor->publish_state(temp);
            }
            break;
          case 0x2b:
            if (dif == 0x04)
            {
              // 1.7.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "1.7.0: %d", temp);
              if (this->instantaneous_power_a_positive_sensor)
                this->instantaneous_power_a_positive_sensor->publish_state(temp);
            }
            break;
          case 0xab:
            if (dif == 0x04 && vife == 0x3c)
            {
              // 2.7.0
              memcpy(&temp, &this->decode_buffer[i], data_len);
              ESP_LOGD(TAG, "2.7.0: %d", temp);
              if (this->instantaneous_power_a_negative_sensor)
                this->instantaneous_power_a_negative_sensor->publish_state(temp);
            }
            break;
          }

          i += data_len;
        }
        if (amisNotOnline)
        {
          amisNotOnline = false;
          ESP_LOGD(TAG, "Data synced");
        }
      }
      else
      {
        ESP_LOGD(TAG, "check bad");
        this->reset_telegram();
      }
    }

    void amis::AMISComponent::reset_telegram()
    {
      log_packet(this->receive_buffer, this->receive_buffer_index);
      this->header_found = false;
      this->footer_found = false;
      this->ack_found = false;
      this->receive_buffer_index = 0;
      this->last_read_time = 0;
      this->expect_frame_length = 0;
      this->ms = millis() + timeout;
      ESP_LOGD(TAG, "Reset Telegram %d", this->ms);
    }

    void amis::AMISComponent::dump_config()
    {
    }

    void amis::AMISComponent::log_packet(uint8_t array[], size_t length)
    {
      char buffer[(length * 3)];

      for (unsigned int i = 0; i < length; i++)
      {
        char nib1 = (array[i] >> 4) & 0x0F;
        char nib2 = (array[i] >> 0) & 0x0F;
        buffer[i * 3] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
        buffer[i * 3 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
        buffer[i * 3 + 2] = ' ';
      }

      buffer[(length * 3) - 1] = '\0';

      ESP_LOGV(TAG, buffer);
    }

    bool amis::AMISComponent::receive_timeout_reached() { return millis() - this->last_read_time > this->receive_timeout; }

    bool amis::AMISComponent::available_within_timeout()
    {
      // Data are available for reading on the UART bus?
      // Then we can start reading right away.
      if (this->available())
      {
        this->last_read_time = millis();
        return true;
      }
      // When we're not in the process of reading a telegram, then there is
      // no need to actively wait for new data to come in.
      if (!header_found || !ack_found)
      {
        return false;
      }
      // A telegram is being read. The smart meter might not deliver a telegram
      // in one go, but instead send it in chunks with small pauses in between.
      // When the UART RX buffer cannot hold a full telegram, then make sure
      // that the UART read buffer does not overflow while other components
      // perform their work in their loop. Do this by not returning control to
      // the main loop, until the read timeout is reached.
      if (this->parent_->get_rx_buffer_size() < this->max_telegram_length)
      {
        while (!this->receive_timeout_reached())
        {
          delay(5);
          if (this->available())
          {
            this->last_read_time = millis();
            return true;
          }
        }
      }
      // No new data has come in during the read timeout? Then stop reading the
      // telegram and start waiting for the next one to arrive.
      if (this->receive_timeout_reached())
      {
        ESP_LOGW(TAG, "Timeout while reading data for telegram");
        this->reset_telegram();
      }
      return false;
    }

    void amis::AMISComponent::loop()
    {
      while (this->available_within_timeout())
      {
        const char c = this->read();
        ESP_LOGV(TAG, "Byte: %x", c);
        ESP_LOGV(TAG, "ack found: %d header found: %d", this->ack_found, this->header_found);
        // Find a new telegram start byte.

        if (!this->ack_found && !this->header_found)
        {
          switch (c)
          {
          case 0x10:
            ESP_LOGD(TAG, "Start byte 0x10 of ack telegram found");
            this->reset_telegram();
            this->ack_found = true;
            break;
          case 0x68:
            ESP_LOGD(TAG, "Start byte 0x68 of encrypted telegram found");
            this->reset_telegram();
            this->header_found = true;
            break;
          default:
            continue;
          }
        }

        // Check for buffer overflow.
        if (this->receive_buffer_index >= this->max_telegram_length)
        {
          this->reset_telegram();
          ESP_LOGE(TAG, "Error: encrypted telegram larger than buffer (%d bytes)", this->max_telegram_length);
          log_packet(this->receive_buffer, this->receive_buffer_index);
          return;
        }

        // Store the byte in the buffer.
        this->receive_buffer[this->receive_buffer_index] = c;
        this->receive_buffer_index++;

        //Check for complete ack telegram
        if (this->ack_found && this->receive_buffer_index >= 5)
        {
          if (memcmp(this->receive_buffer, "\x10\x40\xF0\x30\x16", 5) == 0)
          {
            ESP_LOGD(TAG, "ack'ed frame");
            log_packet(this->receive_buffer, this->receive_buffer_index);
            this->write_byte(0xe5);
            this->reset_telegram();
            continue;
          }
        }

        if(this->header_found && this->expect_frame_length==0 && this->receive_buffer[0] == 0x68 && this->receive_buffer[3] == 0x68)
        {
          ESP_LOGD(TAG, "Frame valid");
          this->expect_frame_length = this->receive_buffer[1] + 6;
        }
          
        //Decode Frame
        if (this->expect_frame_length && this->receive_buffer_index >= this->expect_frame_length)
        {
          ESP_LOGD(TAG, "amis_decode");
          log_packet(this->receive_buffer, this->receive_buffer_index);
          amis_decode();

          this->reset_telegram();
        }
      }
    }
  }
}
