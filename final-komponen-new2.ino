#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <FS.h>
#include <LittleFS.h>
#include <time.h>

// Define semua pin untuk sensor
#define DHTPIN 4
#define DHTTYPE DHT22
#define BUZZER 14
const int relay_pin1 = 25;
const int relay_pin2 = 26;
const int relay_pin3 = 27;
int soil, sensor_analog;
const int soilPin = 35;

// Publish count sensor NPK
int publishedCount = 0;

// Root CA Certificate for HiveMQ
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";

// Informasi koneksi Wi-Fi
const char* ssid = "GrowBotUMS";
const char* password = "dj@ckCUY1.";

// MQTT Server and Port
const char* mqttServer = "661bfd2f711447c5b245d60d55647cc0.s1.eu.hivemq.cloud";
const int mqttPort = 8883;

// topic yang di subscribe
const char* mqtt_topic1 = "sensor/relay1";
const char* mqtt_topic2 = "sensor/relay2";
const char* mqtt_topic3 = "sensor/relay3";
const char* mqtt_data1 = "data/soil1";
const char* mqtt_bird = "sensor/bird";

//nilai default batas kelembapan tanah
int soilMoist = 50;

// Create instances for WiFi and MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Create DHT sensor instance
DHT dht(DHTPIN, DHTTYPE);

// RTOS task handles
TaskHandle_t taskHandleMqtt;
TaskHandle_t taskHandleLittleFS;

void mqttTask(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    vTaskDelay(10);  // Yield task to other tasks
  }
}

void littleFSTask(void *pvParameters) {
  while (true) {
    // Read and publish NPK data from LittleFS every 5 seconds
    static unsigned long lastPublish = 0;
    unsigned long now = millis();
    if (now - lastPublish >= 5000) {
      readAndPublishNPK();
      lastPublish = now;
    }
    vTaskDelay(100);  // Yield task to other tasks
  }
}

void readAndPublishNPK() {
  File file = LittleFS.open("/data.txt", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Reset the published count
  publishedCount = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    parseLine(line);

    // Delay after publishing all 3 values
    if (publishedCount >= 3) {
      delay(5000); // 5 seconds delay
      publishedCount = 0; // Reset count after delay
    }
  }

  file.close();
}

void parseLine(String line) {
  int colonIndex = line.indexOf(':');
  if (colonIndex == -1) {
    return;  // Skip lines without colon
  }

  String key = line.substring(0, colonIndex);
  String value = line.substring(colonIndex + 1);

  value.trim();

  if (key.equals("Nitrogen")) {
    Serial.print("Nitrogen: ");
    Serial.print(value);
    Serial.println(" mg/kg");
    client.publish("sensor/nitrogen", value.c_str());
    publishedCount++;
  } else if (key.equals("Phosphorous")) {
    Serial.print("Phosphorous: ");
    Serial.print(value);
    Serial.println(" mg/kg");
    client.publish("sensor/phosphorous", value.c_str());
    publishedCount++;
  } else if (key.equals("Potassium")) {
    Serial.print("Potassium: ");
    Serial.print(value);
    Serial.println(" mg/kg");
    client.publish("sensor/potassium", value.c_str());
    publishedCount++;
  }
}

// =============================== KODE UNTUK KOMPONEN =============================== //

void waterflow(){
  // water flow code
}

void buzzer() {
  int count = 0;
  while(count < 5){
    digitalWrite(BUZZER, HIGH);
    delay(1000);
    digitalWrite(BUZZER, LOW);
    delay(200);
    count++;
  }
}

void sensorLain(){
  // SENSOR DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  // SENSOR SOIL MOISTURE
  sensor_analog = analogRead(soilPin);
  soil = (100 - ((sensor_analog / 4095.00) * 100));

  if (!isnan(temperature) && !isnan(humidity) && !isnan(soil)) {
    String tempPayload = String(temperature);
    String humPayload = String(humidity);
    String soilPayload = String(soil);

    client.publish("sensor/temperature", tempPayload.c_str());
    client.publish("sensor/humidity", humPayload.c_str());
    client.publish("sensor/soilMoisturize", soilPayload.c_str());

    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Soil Moisture: ");
    Serial.println(soil);

    if(soil < soilMoist){
      digitalWrite(relay_pin1, HIGH);
      delay(7000);
      digitalWrite(relay_pin2, LOW);
    } else{
      digitalWrite(relay_pin1, LOW);
    }
  } else {
    Serial.println("Failed to read from DHT sensor!");
    delay(2000);
  }
}

// ============================================================================================= //

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", "RADZDuino", "Juaragemastik2024")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic1);
      client.subscribe(mqtt_topic2);
      client.subscribe(mqtt_topic3);
      client.subscribe(mqtt_data1);
      client.subscribe(mqtt_bird);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Print received message to serial monitor
  Serial.print("Pesan diterima dari topik : ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  if (String(topic) == mqtt_topic1) {
    if (message == "ON") {
      digitalWrite(relay_pin1, LOW);
      Serial.println("Relay 1 ON");
    } else if (message == "OFF") {
      digitalWrite(relay_pin1, HIGH);
      Serial.println("Relay 1 OFF");
    }
  } else if (String(topic) == mqtt_topic2) {
    if (message == "ON") {
      digitalWrite(relay_pin2, LOW);
      Serial.println("Relay 2 ON");
    } else if (message == "OFF") {
      digitalWrite(relay_pin2, HIGH);
      Serial.println("Relay 2 OFF");
    }
  } else if (String(topic) == mqtt_topic3) {
    if (message == "ON") {
      digitalWrite(relay_pin3, LOW);
      Serial.println("Relay 3 ON");
    } else if (message == "OFF") {
      digitalWrite(relay_pin3, HIGH);
      Serial.println("Relay 3 OFF");
    }
  } else if(String(topic) == mqtt_data1){
    soilMoist = message.toInt();
    Serial.print("Update Soil Moisturize : ");
    Serial.println(soilMoist);
  } else if(String(topic) == mqtt_bird){
    if(message == "ON"){
    buzzer();
    }
  }
}


void setDateTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.println(asctime(&timeinfo));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize DHT sensor
  dht.begin();

  // inisialisasi pin relay
  pinMode(relay_pin1, OUTPUT);
  digitalWrite(relay_pin1, LOW);
  pinMode(relay_pin2, OUTPUT);
  digitalWrite(relay_pin2, LOW);
  pinMode(relay_pin3, OUTPUT);
  digitalWrite(relay_pin3, LOW);

  // Buzzer setup
  pinMode(BUZZER, OUTPUT);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  // Connect to WiFi
  setup_wifi();
  setDateTime();

  // Set CA certificate
  espClient.setCACert(root_ca);

  // Set MQTT server and callback
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  reconnect();

  // Create tasks for MQTT and LittleFS handling
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, &taskHandleMqtt, 1);
  xTaskCreatePinnedToCore(littleFSTask, "LittleFS Task", 4096, NULL, 1, &taskHandleLittleFS, 1);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sensorLain();
  delay(1000);
}
