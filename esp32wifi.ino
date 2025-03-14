#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h> 

// Pin definitions
#define UART1_TX_PIN 4   // GPIO for UART1 TX
#define UART1_RX_PIN 5   // GPIO for UART1 RX
#define UART2_TX_PIN 17  // GPIO for UART2 TX
#define UART2_RX_PIN 16  // GPIO for UART2 RX

// WiFi credentials
const char* ssid = "iPhone";
const char* password = "Aassddff-1234??";
WiFiServer server(80);

// Time configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -5 * 3600;  // Change this for your timezone (e.g., -5 for EST, +1 for CET)
const int daylightOffset_sec = 0;      // Set to 3600 if daylight saving time is in effect

// WiFi connection parameters
const unsigned long WIFI_CONNECT_TIMEOUT = 20000; // 20 seconds timeout for connection attempts
const unsigned long WIFI_RECONNECT_INTERVAL = 30000; // Try to reconnect every 30 seconds
unsigned long lastReconnectAttempt = 0;
bool wifiConnected = false;

// Function to get current timestamp with timezone
String getTimestamp() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "Time unavailable";
  }
  
  char buffer[25]; // Increased buffer size to accommodate more detailed timestamp
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// Function to connect to WiFi with proper timeout
bool connectToWiFi() {
  Serial.println("Attempting to connect to WiFi...");
  
  // First, disconnect if we were connected before
  WiFi.disconnect(true);
  delay(1000);
  
  // Begin connection attempt
  WiFi.begin(ssid, password);
  
  // Set a timeout
  unsigned long startAttemptTime = millis();
  
  // Wait for connection with timeout
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  // Check if connected successfully
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi");
    wifiConnected = false;
    return false;
  }
}

// Function to check and maintain WiFi connection
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      // We were connected but lost connection
      Serial.println("WiFi connection lost!");
      wifiConnected = false;
    }
    
    // Try to reconnect at intervals
    if (millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL) {
      lastReconnectAttempt = millis();
      Serial.println("Attempting to reconnect...");
      connectToWiFi();
    }
  } else if (!wifiConnected) {
    // We've reconnected
    Serial.println("WiFi reconnected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  }
}

// Global variables to store weight and barcode data
String currentWeight = "";
String currentBarcode = "";
bool weightReceived = false;
bool barcodeReceived = false;

// Parse and store data from STM32 - handles both weight and barcode data
bool processSTM32Data(String rawData) {
  String type = rawData.substring(0, 1);
  String payload = rawData.substring(1, rawData.indexOf('\0'));
  
  if(type == "W") {
    // When new weight data is received, clear any previous barcode data
    // to ensure we don't mix data from different measurements
    currentBarcode = "";
    barcodeReceived = false;
    
    // Store the new weight data
    currentWeight = String(payload.toFloat(), 3);
    weightReceived = true;
    Serial.println("Weight data received from STM32: " + currentWeight);
    
    // Send flag to Jetson to notify weight data is received
    Serial2.println("W_READY");
    Serial.println("Sent weight-ready flag to Jetson");
  }
  else if(type == "C") {
    currentBarcode = payload;
    barcodeReceived = true;
    Serial.println("Barcode data received from STM32: " + currentBarcode);
  }
  else {
    Serial.println("Unexpected data type from STM32: " + type);
    return false;
  }
  
  return (weightReceived && barcodeReceived);
}

// Process barcode data from Jetson as a backup/alternative source
bool processJetsonData(String rawData) {

  String type = rawData.substring(0, 1);
  String payload = rawData.substring(1, rawData.indexOf('\0'));
  // Only process and store barcode data if we already have weight data
  // from the current measurement cycle
  if(!weightReceived) {
    Serial.println("Received barcode from Jetson, but no current weight data available - ignoring");
    return false;
  }
  
  // Assuming Jetson sends barcode data directly as string
  if(rawData.length() > 0) {
    currentBarcode = payload;
    barcodeReceived = true;
    Serial.println("Barcode data received from Jetson: " + currentBarcode);
    return (weightReceived && barcodeReceived); // Return true if we have both pieces of data
  } else {
    Serial.println("Empty data received from Jetson");
    return false;
  }
}

// Get combined data as JSON
String getCombinedDataJSON() {
  String json = "{";
  json += "\"timestamp\":\"" + getTimestamp() + "\",";
  json += "\"weight\":" + currentWeight + ",";
  json += "\"barcode\":\"" + currentBarcode + "\"";
  json += "}";
  
  // Reset flags and data after sending to start fresh for next measurement
  weightReceived = false;
  barcodeReceived = false;
  currentWeight = "";
  currentBarcode = "";
  
  return json;
}

// Format data from Jetson - No longer needed as Jetson data is handled directly
// String formatJetsonData(String data) {
//   return "{\"source\":\"jetson\",\"timestamp\":\"" + getTimestamp() + "\",\"data\":\"" + data + "\"}";
// }

void handleClient(WiFiClient &client) {
  Serial.println("\nNew client connected");
  unsigned long timeout = millis() + 60000; // 30-second initial timeout
  const int KEEP_ALIVE_DURATION = 60000;    // 30 seconds
  const int KEEP_ALIVE_INTERVAL = 1000;     // 1-second keep-alive pulses

  // Wait for client request
  while(client.connected() && millis() < timeout) {
    // Check WiFi status regularly
    checkWiFiConnection();
    
    if(client.available()) {
      String request = client.readStringUntil('\r');
      Serial.println("Received request: " + request);

      // Send HTTP headers with extended keep-alive
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: keep-alive");
      client.println("Keep-Alive: timeout=30, max=20");  // 30-second timeout
      client.println();

      // Main data sending loop
      unsigned long dataTimeout = millis() + KEEP_ALIVE_DURATION;
      unsigned long lastKeepAlive = millis();
      
      while(client.connected() && millis() < dataTimeout) {
        // Check WiFi status regularly
        checkWiFiConnection();
        
        bool dataSent = false;
        
        // Check STM32 data for weight and barcode
        if(Serial1.available()) {
          String stm32Data = Serial1.readStringUntil('\0');
          if(stm32Data.length() > 1) {
            // Process weight and barcode data from STM32
            bool dataComplete = processSTM32Data(stm32Data);
            
            // If we have both weight and barcode data, send the combined data
            if(dataComplete) {
              client.println(getCombinedDataJSON());
              dataSent = true;
              Serial.println("Sent combined data from STM32");
            }
          }
        }

        // Check Jetson data for barcode (as backup/alternative)
        if(!barcodeReceived && Serial2.available()) {
          String jetsonData = Serial2.readStringUntil('\n');
          jetsonData.trim();
          if(jetsonData.length() > 0) {
            // Process barcode data from Jetson
            bool dataComplete = processJetsonData(jetsonData);
            
            // If we now have both weight and barcode data, send the combined data
            if(dataComplete) {
              client.println(getCombinedDataJSON());
              dataSent = true;
              Serial.println("Sent combined data with Jetson barcode");
            }
          }
        }

        // Send keep-alive if no data and interval passed
        if(!dataSent && (millis() - lastKeepAlive >= KEEP_ALIVE_INTERVAL)) {
         // client.println("{\"status\":\"active\",\"timestamp\":\"" + getTimestamp() + "\"}");
          lastKeepAlive = millis();
          Serial.println("Sent keep-alive pulse");
        }
        
        delay(10); // Small delay to prevent CPU hogging
      }
      
      break;
    }
    delay(10);
  }
  
  client.stop();
  Serial.println("Client disconnected");
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to start
  
  Serial.println("ESP32 WiFi Server starting...");
  
  // Configure WiFi in station mode
  WiFi.mode(WIFI_STA);
  
  // Initial connection attempt
  connectToWiFi();
  
  // Configure time if connected
  if (wifiConnected) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Time synchronized with NTP server");
  }
  
  // Initialize UARTs
  Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Check and maintain WiFi connection
  checkWiFiConnection();
  
  // Only accept clients if we're connected to WiFi
  if (wifiConnected) {
    WiFiClient client = server.accept();
    if (client) {
      handleClient(client);
    }
  }
  
  delay(10); // Small delay to prevent CPU hogging
}
