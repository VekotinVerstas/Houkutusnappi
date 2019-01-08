/* Houkutusnappi
 * Houkuttelee sykkivän LED-animaation avulla käyttäjää painamaan nappia 
 * ja painettaessa lähettää MQTT-viestin, sekä välkyttää valoa. 
 * Tunnistaa useamman peräkkäisen painalluksen rämppäykseksi
 * ja lähettää siitä MQTT-viestin, sekä välkyttää valoa erilailla.
 * Käyttää viittä pinniä ESPillä.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "settings.h"
#include <FastLED.h>

FASTLED_USING_NAMESPACE
#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    D5
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    4
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

const int buttonPin = D6;    // the number of the pushbutton pin
const int gndPin = D7;      // napin mikrokytkintä varten tehty "maapinni"

// Variables will change:
int painallus = 0; //napin painallusta varten
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long painallusCount = 0;    // rämppäystä varten napin painalluksien laskenta
unsigned long lastPainallusCount = 0; //
int ramppays = 0; //rämppäyksen tunnistusta varten

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void connectWifi() {
  int count = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    count++;
    Serial.print(".");
    if ( count > 20 ) {
      ESP.restart();
    }
  }
  Serial.println("Connected to wifi");
}

void setup() {
  delay(3000); // 3 second delay for recovery
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);
  Serial.begin(115200);
  Serial.println("Ramppaysnappi");

  WiFi.mode(WIFI_STA);
  connectWifi();

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

static void mqtt_send(const char *topic, const char *message)
{
  // Make sure we have wifi and if not try to get some wifi. If we do not have saved wifi settings create accespoint with esp_id and wifi_pw ( at first run login to ap and save wifi settings ).
  if (!mqttClient.connected()) {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.connect("Ramppaysnappi", MQTT_USER, MQTT_PASSWORD);
  }
  if (mqttClient.connected()) {
    Serial.print("Publishing ");
    Serial.print(message);
    Serial.print(" to ");
    Serial.print(topic);
    Serial.print("...");
    int result = mqttClient.publish(topic, message, true);
    Serial.println(result ? "OK" : "FAIL");
  }
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {lepo, valkytys, ramppaysvalkytys};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void loop()
{
  if ( WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }
  else {
    gPatterns[gCurrentPatternNumber]();  //FastLedin toiminnot
    FastLED.show();
    FastLED.delay(1000 / FRAMES_PER_SECOND);

    EVERY_N_SECONDS(4) {             //4 sekunnin välein palataan "perustila"-animaatioon
      gCurrentPatternNumber = 0;
    }

    if ((painallus == 1) and (gCurrentPatternNumber == 0)) { //"painallus"-animaation päällekytkentä
      gCurrentPatternNumber = 1;
      painallus = 0;
    }

    int reading = digitalRead(buttonPin); //Debounce

    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {

      if (reading != buttonState) {
        buttonState = reading;

        if (buttonState == LOW) { //Jos nappia painettu
          painallus = 1;
          painallusCount++;
          mqtt_send(MQTT_TOPIC, "painallus"); //lähetetään MQTT-viesti
        }
      }
    }

    lastButtonState = reading;

    EVERY_N_SECONDS(2) { //Napin rämppäyksestä
      ramppays = 0;
      if (painallusCount - lastPainallusCount > 2) { //"liikapainallusten" tarkkailu
        mqtt_send(MQTT_TOPIC, "ramppays"); //lähetetään MQTT-viesti
        ramppays = 1;
        gCurrentPatternNumber = 2; //vaihdetaan "rämppäys"-animaatioon
      }
      lastPainallusCount = painallusCount; //"liikapainallusten" tarkkailu
    }
  }
}


void lepo() //perustilan "houkutus"-animaatio
{
  int pos = beatsin8(10, 32);
  fill_solid( leds, NUM_LEDS, CHSV(0, 255, pos));
}

void valkytys() //painalluksen animaatio
{
  int pos = beatsin8(150, 16);
  fill_solid( leds, NUM_LEDS, CHSV(0, 255, pos));
}

void ramppaysvalkytys() //rämppäyksestä ajettava animaatio
{
  int pos = beatsin8(240, 16);
  fill_solid( leds, NUM_LEDS, CHSV(pos, pos, pos));
}
