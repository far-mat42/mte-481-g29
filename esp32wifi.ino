#include <WiFi.h>
#include <WiFiClient.h>

#define UART1_TX_PIN 4   // GPIO for UART1 TX
#define UART1_RX_PIN 5   // GPIO for UART1 RX

#define UART2_TX_PIN 17  // GPIO for UART2 TX
#define UART2_RX_PIN 16  // GPIO for UART2 RX

// const char* ssid = "BELL785";    // Your Wi-Fi SSID
// const char* password = "322325C1C46A"; // Your Wi-Fi password

const char* ssid = "iPhone";    // Your Wi-Fi SSID
const char* password = "Aassddff-1234??"; // Your Wi-Fi password

WiFiServer server(80);  // Create a web server on port 80

void setup() {
  // Initialize UART0 for debugging
  Serial.begin(115200);
  // while (!Serial) {
  //   delay(10); // Wait for Serial to initialize
  // }
  // Serial.println("UART0 (Serial) initialized for debugging");

  // Initialize Wifi
 Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();  // Start the server

  // Initialize UART1
  Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Serial.println("UART1 initialized for communication with peripheral 1");

  // Initialize UART2
  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  Serial.println("UART2 initialized for communication with peripheral 2");

  // Test messages
  Serial1.println("Hello from ESP32 on UART1!");
  Serial2.println("Hello from ESP32 on UART2!");
}

void loop() {
// Check for any available client connections
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client Connected.");
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');  // Read client request
        Serial.println(request);
        client.flush();

        // Send HTTP response
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println();

        // Check for data on UART1
        if (Serial1.available()) {
          String data1 = Serial1.readStringUntil('\n');
          client.println("Data from STM32: " + data1);  // Send data to the laptop

          // Echo back to UART1
          Serial1.print("Echo: ");
          Serial1.println(data1);
        }
        delay(10);
        // Check for data on UART2
        if (Serial2.available()) {
          String data2 = Serial2.readStringUntil('\n');
          client.println("Data from Jetson: " + data2);  // Send data to the laptop

          // Echo back to UART2
          Serial2.print("Echo: ");
          Serial2.println(data2);
        }
        break;
      }
 
    client.stop();  // Close the connection when the client disconnects
    Serial.println("Client Disconnected.");
    }
  }
  // Forward data from Serial Monitor to UART1 and UART2
  // if (Serial.available()) {
  //   String debugInput = Serial.readStringUntil('\n');
  //   Serial1.println(debugInput);
  //   Serial2.println(debugInput);
  // }
}