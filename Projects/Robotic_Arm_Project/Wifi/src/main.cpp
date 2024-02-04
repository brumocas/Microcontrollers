#include <WiFi.h>

const char* ssid = "Escritório/Office";
const char* password = "EEDAB1237C";

WiFiServer server(80);

String header;
String picoLEDState = "off";
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

enum Mode {
  MODE1,
  MODE2,
  MODE3,
  MODE4
};
Mode state;

String modeToString(Mode mode) {
  switch (mode) {
    case MODE1:
      return "Mode 1";
    case MODE2:
      return "Mode 2";
    case MODE3:
      return "Mode 3";
    case MODE4:
      return "Mode 4";
    default:
      return "Unknown Mode";
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("WiFi connected at IP Address ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {
        char c = client.read();
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (header.indexOf("GET /led/on") >= 0) {
              Serial.println("LED on");
              picoLEDState = "on";
              digitalWrite(LED_BUILTIN, HIGH);
            } else if (header.indexOf("GET /led/off") >= 0) {
              Serial.println("LED off");
              picoLEDState = "off";
              digitalWrite(LED_BUILTIN, LOW);
            }

            if (header.indexOf("GET /Mode1") >= 0) {
              Serial.println("Mode1 selected");
              state = MODE1;
            }

            if (header.indexOf("GET /Mode2") >= 0) {
              Serial.println("Mode2 selected");
              state = MODE2;
            }

            if (header.indexOf("GET /Mode3") >= 0) {
              Serial.println("Mode3 selected");
              state = MODE3;
            }    

            if (header.indexOf("GET /Mode4") >= 0) {
              Serial.println("Mode4 selected");
              state = MODE4;
            }    
                   
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #F23A3A;}</style></head>");
            client.println("<body><h1>ACE Robot Mode Control</h1>");
            client.println("<p>Led State: " + picoLEDState + "</p>");
            
  

            if (picoLEDState == "off") {
              client.println("<p><a href=\"/led/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/led/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            
            client.println("<p>Current Robot mode is " + modeToString(state) + "</p>");

            if (state == MODE1) {
              //client.println("<p><a href=\"/Mode1\"><button class=\"button\">Mode 1</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode1\"><button class=\"button button2\">Mode 1</button></a></p>");
            }

            if (state == MODE2) {
              //client.println("<p><a href=\"/Mode2\"><button class=\"button\">Mode 2</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode2\"><button class=\"button button2\">Mode 2</button></a></p>");
            }

            if (state == MODE3) {
              //client.println("<p><a href=\"/Mode3\"><button class=\"button\">Mode 3</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode3\"><button class=\"button button2\">Mode 3</button></a></p>");
            }

            if (state == MODE4) {
              //client.println("<p><a href=\"/Mode4\"><button class=\"button\">Mode 4</button></a></p>");
            } else {
              client.println("<p><a href=\"/Mode4\"><button class=\"button button2\">Mode 4</button></a></p>");
            }
            
            client.println("</body></html>");
            client.println();
            break;

          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }

      }
    }
    header = "";
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
