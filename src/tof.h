#include "Adafruit_VL53L0X.h"

typedef struct {
  bool started;      // has the sensor been started
  Adafruit_VL53L0X psensor; // pointer to object
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t last_range;        // range value used in continuous mode stuff.
  uint8_t sensor_status;      // status of the sensor
} tofSensor_t;
