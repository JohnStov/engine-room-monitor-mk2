// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>

#include "sensesp.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/lambda_transform.h"

#include <SensirionI2cScd4x.h>
#include <Wire.h>

using namespace sensesp;

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

static char errorMessage[64];
static int16_t error;
unsigned int co2_sample_interval = 5000;
  
struct scd40_data {
    uint16_t concentration;
    float temperature;
    float relativeHumidity;
};

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engineroom-monitor")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  SensirionI2cScd4x co2_sensor;

  Wire.setPins(16, 17);
  Wire.begin();
  co2_sensor.begin(Wire, SCD40_I2C_ADDR_62);

  error = co2_sensor.wakeUp();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute wakeUp(): %s", errorMessage);
  }
  error = co2_sensor.stopPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute stopPeriodicMeasurement(): %s", errorMessage);
  }
  error = co2_sensor.reinit();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute reinit(): %s", errorMessage);
  }
  // Read out information about the sensor
  uint64_t serialNumber;
  error = co2_sensor.getSerialNumber(serialNumber);
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute getSerialNumber(): %s", errorMessage);
  }
  ESP_LOGI(__FILE__, "serial number: %d", serialNumber);

  error = co2_sensor.startPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute startPeriodicMeasurement(): %s", errorMessage);
  }

  auto co2_callback = [&]() -> scd40_data {
    bool dataReady = false;
    scd40_data result;
    static scd40_data lastGoodResult = {0, 0.0, 0.0};

    error = co2_sensor.getDataReadyStatus(dataReady);
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        ESP_LOGE(__FILE__, "Error trying to execute getDataReadyStatus(): %s");
    }

    if (dataReady)
    {
      error = co2_sensor.readMeasurement(result.concentration, result.temperature, result.relativeHumidity);
      if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        ESP_LOGE(__FILE__, "Error trying to execute readMeasurement(): %s");
      }
      else
        lastGoodResult = result;
        return result;
    }

    return lastGoodResult;
  };

  auto concentrationTransform = new LambdaTransform<scd40_data, uint16_t>([] (scd40_data data) -> uint16_t { return data.concentration; });
  auto temperatureTransform = new LambdaTransform<scd40_data, float>([] (scd40_data data) -> float { return data.temperature + 272.15; });
  auto humidityTransform = new LambdaTransform<scd40_data, float>([] (scd40_data data) -> float { return data.relativeHumidity / 100.0; });

  auto* engine_room_co2 = new RepeatSensor<scd40_data>(co2_sample_interval, co2_callback);
  const char* co2_path = "environment.inside.engineRoom.co2Concentration";
  const char* temperature_path = "environment.inside.engineRoom.temperature";
  const char* humidity_path = "environment.inside.engineRoom.relativeHumidity";
  engine_room_co2->connect_to(concentrationTransform)->connect_to(new SKOutputInt(co2_path));
  engine_room_co2->connect_to(temperatureTransform)->connect_to(new SKOutputFloat(temperature_path));
  engine_room_co2->connect_to(humidityTransform)->connect_to(new SKOutputFloat(humidity_path));

  
    
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}





void loop() { event_loop()->tick(); }
