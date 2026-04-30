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
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"


#include <SensirionI2cScd4x.h>
#include <MICS6814Wrapper.h>
#include <Wire.h>
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp::onewire;

using namespace sensesp;

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

static char errorMessage[64];
static int16_t error;
unsigned int sample_interval = 5000;
  
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
                    //->set_wifi_client("MySSID", "my_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  Wire.setPins(16, 17);
  Wire.begin();


  SensirionI2cScd4x co2Sensor;
  MICS6814Wrapper mics6814(false, 0x18);
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(4);
  
  co2Sensor.begin(Wire, SCD40_I2C_ADDR_62);

  error = co2Sensor.wakeUp();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute wakeUp(): %s", errorMessage);
  }
  error = co2Sensor.stopPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute stopPeriodicMeasurement(): %s", errorMessage);
  }
  error = co2Sensor.reinit();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute reinit(): %s", errorMessage);
  }
  // Read out information about the sensor
  uint64_t serialNumber;
  error = co2Sensor.getSerialNumber(serialNumber);
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute getSerialNumber(): %s", errorMessage);
  }
  ESP_LOGI(__FILE__, "serial number: %d", serialNumber);

  error = co2Sensor.startPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    ESP_LOGE(__FILE__, "Error trying to execute startPeriodicMeasurement(): %s", errorMessage);
  }

  auto co2_callback = [&]() -> scd40_data {
    bool dataReady = false;
    scd40_data result;
    static scd40_data lastGoodResult = {0, 0.0, 0.0};

    error = co2Sensor.getDataReadyStatus(dataReady);
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        ESP_LOGE(__FILE__, "Error trying to execute getDataReadyStatus(): %s");
    }

    if (dataReady)
    {
      error = co2Sensor.readMeasurement(result.concentration, result.temperature, result.relativeHumidity);
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

  auto engine_room_co2 = new RepeatSensor<scd40_data>(sample_interval, co2_callback);

  auto co2_output = new SKOutputInt("environment.inside.engineRoom.co2Concentration", new SKMetadata("ppm", "Engine room CO2 Concentration", "CO2 concentration in parts per million"));
  auto temperature_output = new SKOutputFloat("environment.inside.engineRoom.temperature", new SKMetadata("K", "Engine room temperature"));
  auto humidity_output = new SKOutputFloat("environment.inside.engineRoom.relativeHumidity", new SKMetadata("ratio", "Engine room relative humidity"));
  
  engine_room_co2->connect_to(concentrationTransform)->connect_to(co2_output);
  engine_room_co2->connect_to(temperatureTransform)->connect_to(temperature_output);
  engine_room_co2->connect_to(humidityTransform)->connect_to(humidity_output)->connect_to(new LambdaConsumer<float>([&](float val){ mics6814.set_led(0, 31, 0); delay(200); mics6814.set_led(63, 0, 0); return;}));

  if (!mics6814.init()) {
    ESP_LOGE(__FILE__, "Failed to initialize the gas sensor");
  }

  mics6814.set_heater(true);
  mics6814.set_led(31, 31, 31);

  auto reducingTransform = new LambdaTransform<MICS6814::Reading, float>([] (MICS6814::Reading data) -> uint16_t { return data.reducing; });
  auto nh3Transform = new LambdaTransform<MICS6814::Reading, float>([] (MICS6814::Reading data) -> float { return data.nh3; });
  auto oxidisingTransform = new LambdaTransform<MICS6814::Reading, float>([] (MICS6814::Reading data) -> float { return data.oxidising; });

  auto engine_room_gases = new RepeatSensor<MICS6814::Reading>(sample_interval, [&]()->MICS6814::Reading { return mics6814.read_all(); });
  auto reducing_output = new SKOutputFloat("environment.inside.engineRoom.reducingGases", new SKMetadata("Ohms", "Engine room reducing gases", "A resistance value indicating the relative level of reducing gases (CO) in the atmosphere"));
  auto nh3_output = new SKOutputFloat("environment.inside.engineRoom.nh3", new SKMetadata("Ohms", "Engine room NH3 level", "A resistance value indicating the relative level of Ammonia in the atmosphere"));
  auto oxidising_output = new SKOutputFloat("environment.inside.engineRoom.oxidisingGases", new SKMetadata("Ohms", "Engine room oxidising gases", "A resistance value indicating the relative level of oxidising gases (NO2) in the atmosphere"));
  engine_room_gases->connect_to(reducingTransform)->connect_to(reducing_output);
  engine_room_gases->connect_to(nh3Transform)->connect_to(nh3_output);
  engine_room_gases->connect_to(oxidisingTransform)->connect_to(oxidising_output);

  // Measure exhaust temperature
  auto exhaust_temp =
    new OneWireTemperature(dts, sample_interval, "/exhaustTemperature/oneWire");
  ConfigItem(exhaust_temp)->set_title("Exhaust Temperature Sensor")->set_sort_order(2000);
  auto exhaust_temp_calibration =
      new Linear(1.0, 0.0, "/exhaustTemperature/calibration");
  ConfigItem(exhaust_temp_calibration)->set_title("Exhaust Temperature Calibration")->set_sort_order(1000);
  auto exhaust_temp_sk_output= new SKOutputFloat(
      "propulsion.port.exhaustTemperature", new SKMetadata("K", "Exhaust Temperature"));

  exhaust_temp->connect_to(exhaust_temp_calibration)
      ->connect_to(exhaust_temp_sk_output);

  // Measure alternator temperature
  auto alternator_temp =
      new OneWireTemperature(dts, sample_interval, "/alternatorTemperature/oneWire");
  ConfigItem(alternator_temp)->set_title("Alternator Temperature Sensor")->set_sort_order(2100);

  auto alternator_temp_calibration =
      new Linear(1.0, 0.0, "/alternatorTemperature/calibration");
  ConfigItem(alternator_temp_calibration)->set_title("Alternator Temperature Calibration")->set_sort_order(1100);
  auto alternator_temp_sk_output = new SKOutputFloat(
      "electrical.alternators.0.temperature", new SKMetadata("K", "Alternator Temperature"));

  alternator_temp->connect_to(alternator_temp_calibration)
      ->connect_to(alternator_temp_sk_output);

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
