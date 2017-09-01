#include <Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <Hash.h>

#include "Filter.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

#define servoPin 2
#define LED_RED     15
#define RECOVER_MINALT 0.5
#define RECOVER_LAUNCH 1.0

#define SAMPLING_TIME 50
#define COMM_TIME 500

Adafruit_BMP280 bmp; // I2C

Servo myservo;  // create servo object to control a servo
FloatBuffer altRefBuf(10);
FloatBuffer altShortBuf(5);
FloatBuffer altFlightBuf(200);

int wifiStatus = WL_DISCONNECTED;
int aktuellZaehler = 0;
unsigned long remTime = 100000;
unsigned long startCountdownAt = 0;
unsigned long lastComm = 0;
unsigned long liftOffAt = 0;
float aRef = 0, aMax = 0, curAlt = 0, aMean = 0;
char data = 0;
bool recoverActive = false;
bool bmpOk = false;
bool recoverLaunched = false;
bool liftOff = false;
int flightBufPos = 0;

#define USE_SERIAL Serial

const char WiFiAPPSK[] = "sparkfun";

char *charBuffer = new char[200];

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload,
		size_t length) {

	switch (type) {
	case WStype_DISCONNECTED:
		USE_SERIAL.printf("[%u] Disconnected!\n", num);
		break;
	case WStype_CONNECTED: {
		IPAddress ip = webSocket.remoteIP(num);
		USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num,
				ip[0], ip[1], ip[2], ip[3], payload);
	}
		break;
	case WStype_TEXT:
		USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

		if (payload[0] == '#') {
			// we get RGB data

			// decode rgb data
			uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL,
					16);

			/*
			 analogWrite(LED_RED,    ((rgb >> 16) & 0xFF));
			 analogWrite(LED_GREEN,  ((rgb >> 8) & 0xFF));
			 analogWrite(LED_BLUE,   ((rgb >> 0) & 0xFF));
			 */
		}

		break;
	}

}

void setup() {

  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, 0);
	myservo.attach(servoPin); // attaches the servo on pin 9 to the servo object

	Serial.begin(9600);

	bmpOk = bmp.begin();
  Serial.print("BMP: ");
  Serial.println(bmpOk);
}

void launchRecover() {
	Serial.println("GO!!!");
	myservo.write(map(800, 0, 1023, 0, 180));
	recoverActive = false;
	recoverLaunched = true;
}

void prepareLaunch() {
	Serial.println("prepareLaunch");
	startCountdownAt = millis();
	aMax = 0;
	recoverActive = false;
	liftOff = false;
	altFlightBuf.reset();
	flightBufPos = 0;
}

void reset() {
	Serial.println("RESET");
	aMax = 0;
	recoverActive = false;
	myservo.write(map(0, 0, 1023, 0, 180));
	startCountdownAt = 0;
	recoverLaunched = false;
	liftOff = false;
}

void sendFlightBuf() {

	for (int i = 0; i < altFlightBuf.getSize(); i += 2) {
		Serial.print("FlightTime");
		Serial.print(altFlightBuf.getVal(i));
		Serial.print("H");
		Serial.println(altFlightBuf.getVal(i + 1));
	}
}

void doControl() {

	unsigned long currentMillis = millis();

	webSocket.loop();

	if (Serial.available()) {
		data = Serial.read();
		switch (data) {
		case 'r':
			reset();
			break;
		case 'l':
			launchRecover();
			break;
		case 'x':
			prepareLaunch();
			break;
		case 'f':
			sendFlightBuf();
			break;
		default:
			if (data < '0' || data > '9')
				break;
			startCountdownAt = 0;
			aktuellZaehler = data - 48;
			// 1--> '49'
			Serial.print("set timer: ");
			Serial.println(aktuellZaehler);
		}
	}

	if (aktuellZaehler > 0 && startCountdownAt > 0
			&& (currentMillis > startCountdownAt + aktuellZaehler * 1000)
			&& !recoverLaunched) {
		launchRecover();
	}

	delay(SAMPLING_TIME);

	altShortBuf.addVal(bmp.readAltitude(1017.25));
	aMean = altShortBuf.getMean();
	curAlt = aMean - aRef;
	if (!recoverActive && startCountdownAt > 0 && curAlt > RECOVER_MINALT) {
		recoverActive = true;
		Serial.print("Activate recover");
	}

	if (recoverActive && curAlt < aMax - RECOVER_LAUNCH) {
		launchRecover();
	}

	if (curAlt > aMax)
		aMax = curAlt;

	if (startCountdownAt <= 0) {
		altRefBuf.addVal(aMean);
		aRef = altRefBuf.getMean();
		aMax = 0;
	}

	if (aMax > 0.2) {
		liftOff = true;
		liftOffAt = millis();
	}

	if ((startCountdownAt > 0) && (flightBufPos >= 0)) {
		//altFlightBuf.addVal((double)(millis() - startCountdownAt) / 1000);
		flightBufPos = altFlightBuf.addVal(curAlt);
		if (flightBufPos == altFlightBuf.getSize() - 1) {
			flightBufPos = -1;
		}
	}

	if (millis() - lastComm > COMM_TIME) {
		lastComm = millis();

		notifyClient();
	}
}

void loop() {

	if (checkWiFi()) {
		doControl();
	} else {
		delay(5000);
	}

}

void notifyClient() {

	long remMillis = startCountdownAt + aktuellZaehler * 1000 - millis();

	StaticJsonBuffer < 200 > jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["time"] = millis();

	if (!liftOff && (remMillis >= 0)) {
		root["Countdown"] = remMillis;
	}
	root["Altimeter"] = bmpOk;
	root["Timer"] = aktuellZaehler;
	root["Flug"] = liftOff;
	root["Fallschirm aktiv"] = recoverActive;
	root["Fallhöhe"] = aMax - curAlt;
	root["Alt"] = aMean;
	root["H"] = curAlt;
	root["Ref Alt"] = aRef;
	root["Max Alt"] = aMax;

	size_t written = root.printTo(charBuffer, 200);
	webSocket.broadcastTXT(charBuffer, written);

}

void setupSockets() {
	// start webSocket server
	webSocket.begin();
	webSocket.onEvent(webSocketEvent);

	if (MDNS.begin("esp8266")) {
		USE_SERIAL.println("MDNS responder started");
	}

	// Add service to MDNS
	//MDNS.addService("http", "tcp", 80);
	MDNS.addService("ws", "tcp", 81);
}

boolean setupWifiHome() {
	char pass[] = "nfJUr4).kzS3*A";
	char ssid[] = "NETGEAR45";

	Serial.print("Trying to connect to ");
	Serial.println(ssid);

// attempt to connect to Wifi network:
	wifiStatus = WiFi.begin(ssid, pass);

	wifiStatus = WiFi.waitForConnectResult();
	if (wifiStatus != WL_CONNECTED) {
		Serial.println("Connection Failed");
	}

  Serial.println("Connected.");
Serial.print("MAC Addr: ");
Serial.println(WiFi.macAddress());
Serial.print("IP Addr:  ");
Serial.println(WiFi.localIP());
Serial.print("Subnet:   ");
Serial.println(WiFi.subnetMask());
Serial.print("Gateway:  ");
Serial.println(WiFi.gatewayIP());
Serial.print("DNS Addr: ");
Serial.println(WiFi.dnsIP());
Serial.print("Channel:  ");
Serial.println(WiFi.channel());
Serial.print("Status: ");
Serial.println(WiFi.status());
  setupSockets();

  if( wifiStatus == WL_CONNECTED)
    digitalWrite(LED_RED, 1);
  else
    digitalWrite(LED_RED, 0);
	return wifiStatus == WL_CONNECTED;
}

void setupWiFiAP() {
	WiFi.mode(WIFI_AP);

	// Do a little work to get a unique-ish name. Append the
	// last two bytes of the MAC (HEX'd) to "Thing-":
	uint8_t mac[WL_MAC_ADDR_LENGTH];
	WiFi.softAPmacAddress(mac);
	String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX)
			+ String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
	macID.toUpperCase();
	String AP_NameString = "ESP8266 Thing " + macID;

	char AP_NameChar[AP_NameString.length() + 1];
	memset(AP_NameChar, 0, AP_NameString.length() + 1);

	for (int i = 0; i < AP_NameString.length(); i++)
		AP_NameChar[i] = AP_NameString.charAt(i);

	WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

bool checkWiFi() {
	if (wifiStatus == WL_CONNECTED)
		return true;
	return setupWifiHome();
}
