int instron_new_file_pin = 7;       // Instron digital output 1
int instron_active_logging_pin = 8; // Instron digital output 2
int pressure_sensor_pin = A0;
unsigned long start_time = 0;

bool last_logging = 0;

void setup() {
  pinMode(instron_new_file_pin, INPUT);
  pinMode(instron_active_logging_pin, INPUT);
  
  Serial.begin(115200);
}

void loop() {
  // **Boolean flags for controlling the logging state from the Instron software**
  // Indicate test is active and a new file should be created
  bool new_logging = digitalRead(instron_new_file_pin); 
  
  // Indicate the current segment of the test should log pressures.
  bool active_logging = digitalRead(instron_active_logging_pin); 

  // Detect the start of a new logging sesison: "rising edge" of the new_file pin.
  if ((last_logging == 0) && (new_logging == 1)) {
    start_time = millis();
    Serial.println("Logging start");
  } else if ((last_logging == 1) && (new_logging == 0)) {
    Serial.println("Logging end");
  }

  // Read the pressure, and print it if we are logging
  float pressure_volts = analogRead(pressure_sensor_pin) / 1024. * 5;
  float pressure = v_to_pressure(pressure_volts);
  if (new_logging && active_logging) {
    unsigned long t = millis() - start_time;
    Serial.println(String(t) + "," + String(pressure));
  }

  last_logging = new_logging;
  delay(10);
}

float v_to_pressure(float voltage) {
  // Inversion of the pressure-to-voltage formula found in the datasheet:
  // See pg. 11 of https://media.digikey.com/pdf/Data%20Sheets/Honeywell%20PDFs/ssc_series_DS.pdf
  // Constants P_MAX and P_MIN are for SSCDRRN030PGAB5:

  float P_MAX = 30; // psi
  float P_MIN = 0;  // psi
  float V_SUPPLY = 5;

  // The formula says it's 0.1*V_SUPPLY but in practice it seems like the offset is only 0.05 * V_supply???
  // I did some benchmarking of this formula against our digital pressure sensor (Bill - 7/12/2023)
  float pressure = (P_MAX - P_MIN) / (0.8 * V_SUPPLY) * (voltage - 0.05 * V_SUPPLY) + P_MIN;
  return pressure;
}
