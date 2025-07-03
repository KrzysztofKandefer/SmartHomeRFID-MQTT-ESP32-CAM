#include <WiFi.h>
#include <PubSubClient.h>

// === KONFIGURACJA WiFi ===
// const char* ssid = "UPC5647830";
// const char* password = "jvrHrjh4Cynt";
const char *ssid = "wifi_hotspot_123";
const char *password = "12345678";

// === KONFIGURACJA MQTT ===
const char* mqtt_server = "test.mosquitto.org";  // lub IP Twojego brokera
WiFiClient espClient;
PubSubClient client(espClient);

// === UART z STM32 ===
#define RX_PIN 17   // Dopasuj do swojego ESP32-C6
#define TX_PIN 16   // Nie używamy, ale musi być zadeklarowane
HardwareSerial SerialSTM(1);  // UART1


String incomingData = "";

void setup_wifi() {
  Serial.println("Łączenie z WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("Połączono z WiFi.");
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Łączenie z MQTT...");
    if (client.connect("ESP32Client2", "", "")) {
      Serial.println("Połączono.");
      client.subscribe("kandefer/rfid/button");  // <-- Subskrybuj temat
      Serial.print("MQTT state: ");
      Serial.println(client.state());
    } else {
      Serial.println("Błąd połączenia.");
      Serial.print("MQTT state: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Zamień payload na String
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Odebrano z MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Reakcja na przycisk
  if (String(topic) == "kandefer/rfid/button" && message == "1") {
    Serial.println("Przycisk MQTT został naciśnięty!");
    
    // Wyślij polecenie do STM32
    SerialSTM.println("o");  // Możesz to zmienić na dowolny format
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) { ; } // czekaj na połączenie portu szeregowego
  Serial.println("Start programu");
  SerialSTM.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // UART z STM32

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  client.setCallback(callback);  // <-- Powiązanie funkcji callback
}
  
int toggleFlag = 0;

void loop() {
  if (!client.connected()) {
    Serial.println("MQTT niepołączony, ponawiam..."); // Sprawdzenie poprawności połaczenia
    reconnect_mqtt();
  }
  client.loop();

  while (SerialSTM.available()) {
    char c = SerialSTM.read();

    if (c == '\n') {  
      if (toggleFlag == 0){   // Wysłanie nazwy przypisanej do tagu RFID
        client.publish("kandefer/rfid/name", incomingData.c_str());
        toggleFlag = !toggleFlag;
        if (incomingData =! "Unauthorized user\r\n"){
          client.publish("kandefer/rfid/led", String(toggleFlag).c_str());
        }
        Serial.print("Wysłano UID: ");
        Serial.println(incomingData);
        incomingData = "";
      } else{   // Wysyłanie UID tagu RFID
        client.publish("kandefer/rfid/uid", incomingData.c_str());
        toggleFlag = !toggleFlag;
        delay(4000);
        client.publish("kandefer/rfid/led", String(toggleFlag).c_str());
        Serial.print("Wysłano Name: ");
        Serial.println(incomingData);
        incomingData = "";
      }
    } else if (incomingData == "_o_"){  // Wysałanie potwierdzenia otworzenia drzwi
        client.publish("kandefer/rfid/led", "1");
        Serial.print("Drzwi otwarto z aplikacji");
        delay(4000);
        client.publish("kandefer/rfid/led", "0");
        client.publish("kandefer/camera/button", "0");
        incomingData = "";
    } else {
      incomingData += c;
    }
  }

}
