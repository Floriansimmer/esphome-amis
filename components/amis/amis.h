#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

static const char *TAG = "amis";
static const char *AMIS_VERSION = "0.0.1";

namespace esphome
{
  namespace amis
  {

    class AMISComponent : public Component, public uart::UARTDevice
    {
    public:
      void setup() override;
      void loop() override;

      void hex2bin(const std::string s, uint8_t *buf);
      uint8_t dif2len(uint8_t dif);
      void reset_telegram();
      void dump_config() override;
      void amis_decode();
      void set_power_grid_key(const std::string &power_grid_key);
      
      void set_energy_a_positive_sensor(sensor::Sensor *sensor)
      {
        this->energy_a_positive_sensor = sensor;
      }
      void set_energy_a_negative_sensor(sensor::Sensor *sensor)
      {
        this->energy_a_negative_sensor = sensor;
      }
      void set_reactive_energy_a_positive_sensor(sensor::Sensor *sensor)
      {
        this->reactive_energy_a_positive_sensor = sensor;
      }
      void set_reactive_energy_a_negative_sensor(sensor::Sensor *sensor)
      {
        this->reactive_energy_a_negative_sensor = sensor;
      }
      void set_instantaneous_power_a_positive_sensor(sensor::Sensor *sensor)
      {
        this->instantaneous_power_a_positive_sensor = sensor;
      }
      void set_instantaneous_power_a_negative_sensor(sensor::Sensor *sensor)
      {
        this->instantaneous_power_a_negative_sensor = sensor;
      }
      void set_reactive_instantaneous_power_a_positive_sensor(sensor::Sensor *sensor)
      {
        this->reactive_instantaneous_power_a_positive_sensor = sensor;
      }
      void set_reactive_instantaneous_power_a_negative_sensor(sensor::Sensor *sensor)
      {
        this->reactive_instantaneous_power_a_negative_sensor = sensor;
      }
      void set_timestamp_sensor(sensor::Sensor *sensor)
      {
        this->timestamp_sensor = sensor;
      }

      float get_setup_priority() const override { return setup_priority::DATA; }

    private:
      unsigned long lastRead = 0; // Timestamp when data was last read
      int readTimeout = 100;      // Time to wait after last byte before considering data complete
      void log_packet(uint8_t array[], size_t length);

    protected:
      int receive_buffer_index = 0;               // Current position of the receive buffer
      const static int receive_buffer_size = 256; // Size of the receive buffer
      uint8_t receive_buffer[receive_buffer_size]; // Stores the packet currently being received
      uint8_t decode_buffer[128];                // Stores the packet currently being decoded

      /// Wait for UART data to become available within the read timeout.
      ///
      /// The smart meter might provide data in chunks, causing available() to
      /// return 0. When we're already reading a telegram, then we don't return
      /// right away (to handle further data in an upcoming loop) but wait a
      /// little while using this method to see if more data are incoming.
      /// By not returning, we prevent other components from taking so much
      /// time that the UART RX buffer overflows and bytes of the telegram get
      /// lost in the process.
      bool available_within_timeout();

      // Read telegram
      uint32_t receive_timeout=10000;
      bool receive_timeout_reached();
      uint32_t last_read_time=0;
      bool ack_found=false;
      bool header_found=false;
      bool footer_found=false;
      size_t max_telegram_length=128;

      int expect_frame_length = 0;
      uint8_t iv[16];
      uint8_t key[16];
      uint32_t ms = 0;
      bool amisNotOnline = true;
      sensor::Sensor *energy_a_positive_sensor{nullptr};
      sensor::Sensor *energy_a_negative_sensor{nullptr};
      sensor::Sensor *reactive_energy_a_positive_sensor{nullptr};
      sensor::Sensor *reactive_energy_a_negative_sensor{nullptr};
      sensor::Sensor *instantaneous_power_a_positive_sensor{nullptr};
      sensor::Sensor *instantaneous_power_a_negative_sensor{nullptr};
      sensor::Sensor *reactive_instantaneous_power_a_positive_sensor{nullptr};
      sensor::Sensor *reactive_instantaneous_power_a_negative_sensor{nullptr};
      sensor::Sensor *timestamp_sensor{nullptr};
    };

  } // namespace rdm6300
} // namespace esphome
