/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will create a new access point (with no password).
  It will then launch a new server and print out the IP address
  to the Serial monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 13.

  If the IP address of your board is yourAddress:
    http://yourAddress/H turns the LED on
    http://yourAddress/L turns it off

  created 25 Nov 2012
  by Tom Igoe
  adapted to WiFi AP by Adafruit
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include "pin_config.h" // Configure the pins used for the ESP32 connection
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
uint8_t airliftRed = 0;
uint8_t airliftGreen = 0;
uint8_t airliftBlue = 0;

void printWiFiStatus();
const char *wlStatusToString(int s);
void setAirliftAll(bool on);
void setAirliftRgb(uint8_t r, uint8_t g, uint8_t b);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);      // set the LED pin mode

  // Set up the pins!
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  Serial.print("Pin map SS/BUSY/RESET/GPIO0: ");
  Serial.print(SPIWIFI_SS);
  Serial.print("/");
  Serial.print(SPIWIFI_ACK);
  Serial.print("/");
  Serial.print(ESP32_RESETN);
  Serial.print("/");
  Serial.println(ESP32_GPIO0);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version: ");
  Serial.println(fv);
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address of will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);
  Serial.print("AP password length: ");
  Serial.println(strlen(pass));

  // WPA2 AP requires 8..63 chars. For shorter passwords, use open AP for testing.
  if (strlen(pass) >= 8 && strlen(pass) <= 63) {
    status = WiFi.beginAP(ssid, pass);
    Serial.println("Attempting WPA2 AP...");
  } else {
    Serial.println("Invalid WPA2 password length (must be 8..63).");
    Serial.println("Falling back to OPEN AP for diagnostics.");
    status = WiFi.beginAP(ssid);
  }

  Serial.print("beginAP() status: ");
  Serial.print(status);
  Serial.print(" (");
  Serial.print(wlStatusToString(status));
  Serial.println(")");

  // AP can report either listening (no client yet) or connected (client already joined)
  if (status != WL_AP_LISTENING && status != WL_AP_CONNECTED) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // Quick LED self-test on AirLift RGB LEDs
  setAirliftAll(false);
  delay(150);
  setAirliftAll(true);
  delay(150);
  setAirliftAll(false);

  // you're connected now, so print out the status
  printWiFiStatus();
}


void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");
            client.print("Click <a href=\"/AON\">here</a> turn AirLift LEDs ON<br>");
            client.print("Click <a href=\"/AOFF\">here</a> turn AirLift LEDs OFF<br>");
            client.print("Click <a href=\"/RON\">here</a> red ON<br>");
            client.print("Click <a href=\"/ROFF\">here</a> red OFF<br>");
            client.print("Click <a href=\"/GON\">here</a> green ON<br>");
            client.print("Click <a href=\"/GOFF\">here</a> green OFF<br>");
            client.print("Click <a href=\"/BON\">here</a> blue ON<br>");
            client.print("Click <a href=\"/BOFF\">here</a> blue OFF<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /AON")) {
          setAirliftAll(true);
        }
        if (currentLine.endsWith("GET /AOFF")) {
          setAirliftAll(false);
        }
        if (currentLine.endsWith("GET /RON")) {
          airliftRed = 255;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
        if (currentLine.endsWith("GET /ROFF")) {
          airliftRed = 0;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
        if (currentLine.endsWith("GET /GON")) {
          airliftGreen = 255;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
        if (currentLine.endsWith("GET /GOFF")) {
          airliftGreen = 0;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
        if (currentLine.endsWith("GET /BON")) {
          airliftBlue = 255;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
        if (currentLine.endsWith("GET /BOFF")) {
          airliftBlue = 0;
          setAirliftRgb(airliftRed, airliftGreen, airliftBlue);
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}

const char *wlStatusToString(int s) {
  switch (s) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    case WL_AP_LISTENING: return "WL_AP_LISTENING";
    case WL_AP_CONNECTED: return "WL_AP_CONNECTED";
    case WL_AP_FAILED: return "WL_AP_FAILED";
    default: return "UNKNOWN";
  }
}

void setAirliftAll(bool on) {
  if (on) {
    airliftRed = 255;
    airliftGreen = 255;
    airliftBlue = 255;
    WiFi.setLEDs(airliftRed, airliftGreen, airliftBlue);
    Serial.println("AirLift LEDs: ON");
  } else {
    airliftRed = 0;
    airliftGreen = 0;
    airliftBlue = 0;
    WiFi.setLEDs(airliftRed, airliftGreen, airliftBlue);
    Serial.println("AirLift LEDs: OFF");
  }
}

void setAirliftRgb(uint8_t r, uint8_t g, uint8_t b) {
  airliftRed = r;
  airliftGreen = g;
  airliftBlue = b;
  WiFi.setLEDs(airliftRed, airliftGreen, airliftBlue);
  Serial.print("AirLift LEDs RGB = ");
  Serial.print(airliftRed);
  Serial.print(",");
  Serial.print(airliftGreen);
  Serial.print(",");
  Serial.println(airliftBlue);
}
