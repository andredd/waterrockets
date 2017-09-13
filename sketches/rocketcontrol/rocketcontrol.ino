#include <Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
//#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <Hash.h>
#include "MPU6050_6Axis_MotionApps20.h"

#include "Filter.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

#define servoPin 3
#define LED_PIN     2
#define RECOVER_MINALT 0.5
#define RECOVER_LAUNCH 1.0

#define SAMPLING_TIME_BMP 50
#define COMM_TIME 9999999

int16_t ax, ay, az;
int16_t gx, gy, gz;

SimpleBuffer<int16_t> bufAx(20),
  bufAy(20),
  bufAz(20),
  bufgx(20),
  bufgy(20),
  bufgz(20);

  /*
int MPU6050_ACCEL_OFFSET_X = -3009,
  MPU6050_ACCEL_OFFSET_Y = -4149,
  MPU6050_ACCEL_OFFSET_Z = 1536,
  MPU6050_GYRO_OFFSET_X  = 74,
  MPU6050_GYRO_OFFSET_Y  = -24,
  MPU6050_GYRO_OFFSET_Z = 34;
*/
int MPU6050_ACCEL_OFFSET_X = 0,
  MPU6050_ACCEL_OFFSET_Y = 0,
  MPU6050_ACCEL_OFFSET_Z = 0,
  MPU6050_GYRO_OFFSET_X  = 0,
  MPU6050_GYRO_OFFSET_Y  = 0,
  MPU6050_GYRO_OFFSET_Z = 0;


#define INTERRUPT_PIN 13  // use pin 2 on Arduino Uno & most boards

Adafruit_BMP280 bmp; // I2C
MPU6050 mpu;
Servo myservo;  // create servo object to control a servo
SimpleBuffer<float> altRefBuf(10),
  altShortBuf(5),
  altFlightBuf(200);

int wifiStatus = WL_DISCONNECTED;
int aktuellZaehler = 9;
unsigned long remTime = 100000;
unsigned long startCountdownAt = 0;
unsigned long lastComm = 0, lastSampleBmp = 0;
unsigned long liftOffAt = 0;
unsigned long currentMillis = 0;
float aRef = 0, aMax = 0, curAlt = 0, aMean = 0;
bool recoverActive = false;
bool bmpOk = false;
bool recoverLaunched = false;
bool liftOff = false;
int flightBufPos = 0;

#define USE_SERIAL Serial

const char WiFiAPPSK[] = "sparkfun";

char *charBuffer = new char[100];

WebSocketsServer webSocket = WebSocketsServer(81);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool blinkState = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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
			onTextCommand(payload[1]);
		}

		break;
	}

}

void setup() {

  Serial.begin(38400);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  doBlink(2, 300);
	myservo.attach(servoPin); // attaches the servo on pin 9 to the servo object

  // vorsichtshalber nach reboot auslösen
  launchRecover();
  
	bmpOk = bmp.begin();
  Serial.print("BMP: ");
  Serial.println(bmpOk);
  if( bmpOk) 
    doBlink(1, 300);
  else
    doBlink(3, 300);

  initMpu();
}

void calibrateMpu(){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    bufAx.addVal(ax);
    bufAy.addVal(ay);
    bufAz.addVal(az);

    bufgx.addVal(gx);
    bufgy.addVal(gy);
    bufgz.addVal(gz);

    Serial.print(bufAx.getMean());
    Serial.print(", ");
    Serial.print(bufAy.getMean());
    Serial.print(", ");
    Serial.print(bufAz.getMean());
    Serial.print(", ");
    Serial.print(bufgx.getMean());
    Serial.print(", ");
    Serial.print(bufgy.getMean());
    Serial.print(", ");
    Serial.println(bufgz.getMean());

    double corr = -bufAx.getMean();
    if( corr < -1 || corr > 1){
      MPU6050_ACCEL_OFFSET_X += (corr > 0 ? 1 : -1);
      mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    }

    corr = -bufAy.getMean();
    if( corr < -1 || corr > 1) {
      MPU6050_ACCEL_OFFSET_Y += (corr > 0 ? 1 : -1);
      mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    }

    corr = -bufAz.getMean();
    if( corr < -1 || corr > 1) {
      MPU6050_ACCEL_OFFSET_Z += (corr > 0 ? 1 : -1);
      mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    }

    corr = -bufgx.getMean();
    if( corr < -1 || corr > 1) {
      MPU6050_GYRO_OFFSET_X += (corr > 0 ? 1 : -1);
      mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    }
    corr = -bufgy.getMean();
    if( corr < -1 || corr > 1) {
      MPU6050_GYRO_OFFSET_Y += (corr > 0 ? 1 : -1);
      mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    }
    corr = -bufgz.getMean();
    if( corr < -1 || corr > 1) {
      MPU6050_GYRO_OFFSET_Z += (corr > 0 ? 1 : -1);
      mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
    }
}
  
  
void initMpu(){
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    if( mpu.testConnection())
    {
      doBlink(5, 300);
      delay(2000);
      Serial.println(F("MPU6050 connection successful"));
    }
    else {
      doBlink(1, 300);
      delay(2000);
      Serial.println(F("MPU6050 connection failed"));
    }

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void launchRecover() {
  notifyClient("GO!!!");
	myservo.write(map(800, 0, 1023, 0, 180));
	recoverActive = false;
	recoverLaunched = true;
}

void prepareLaunch() {
	notifyClient("prepareLaunch");
	startCountdownAt = millis();
	aMax = 0;
	recoverActive = false;
	liftOff = false;
	altFlightBuf.reset();
	flightBufPos = 0;
}

void reset() {
	notifyClient("RESET");
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

void onTextCommand(char cmd){

    switch (cmd) {
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
      if (cmd < '0' || cmd > '9')
        break;
      startCountdownAt = 0;
      aktuellZaehler = cmd - 48;
      // 1--> '49'
      notifyClient("set timer: ");
      Serial.println(aktuellZaehler);
    }
}

void sampleBmp(){
  if( lastSampleBmp + SAMPLING_TIME_BMP > currentMillis ) return;

  lastSampleBmp = currentMillis;

  altShortBuf.addVal(bmp.readAltitude(1017.25));
  aMean = altShortBuf.getMean();
  curAlt = aMean - aRef;

  if (curAlt > aMax)
    aMax = curAlt;

  if (startCountdownAt <= 0) {
    altRefBuf.addVal(aMean);
    aRef = altRefBuf.getMean();
    aMax = 0;
  }

  if ((startCountdownAt > 0) && (flightBufPos >= 0)) {
    //altFlightBuf.addVal((double)(millis() - startCountdownAt) / 1000);
    flightBufPos = altFlightBuf.addVal(curAlt);
    if (flightBufPos == altFlightBuf.getSize() - 1) {
      flightBufPos = -1;
    }
  }

}

void processMpu(){
      // if programming failed, don't try to do anything
    if (!dmpReady) return;

    if( !mpuInterrupt && fifoCount < packetSize) return;
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        if( !liftOff ){
          calibrateMpu();
        } else 
        {
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

                 Serial.print(aaReal.x);
        Serial.print(", ");
        Serial.print(aaReal.y);
        Serial.print(", ");
        Serial.println(aaReal.z);
    
        }
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  
void doControl() {
	currentMillis = millis();

	webSocket.loop();

	if (aktuellZaehler > 0 && startCountdownAt > 0
			&& (currentMillis > startCountdownAt + aktuellZaehler * 1000)
			&& !recoverLaunched) {
		launchRecover();
	}

  sampleBmp();

	if (!recoverActive && startCountdownAt > 0 && curAlt > RECOVER_MINALT) {
		recoverActive = true;
		notifyClient("Activate recover");
	}

	if (recoverActive && curAlt < aMax - RECOVER_LAUNCH) {
		launchRecover();
	}

	if (aMax > 0.2) {
		liftOff = true;
		liftOffAt = currentMillis;
	}

  processMpu();
  
	if (currentMillis - lastComm > COMM_TIME) {
		lastComm = currentMillis;

		sendDataToClient();
	}
}

void loop() {

	if (checkWiFi()) {
		doControl();
	} else {
    doBlink(2, 500);
		delay(2000);
	}

}

void sendDataToClient() {

	long remMillis = startCountdownAt + aktuellZaehler * 1000 - millis();

/*
	StaticJsonBuffer < 100 > jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root["time"] = millis();

	if (!liftOff && (remMillis >= 0)) {
		root["Countdown"] = remMillis;
	}
	root["Altimeter"] = bmpOk;
	root["Timer"] = aktuellZaehler;
	root["Flug"] = liftOff;
	root["Fallschirm aktiv"] = recoverActive;
	root["Fallhoehe"] = aMax - curAlt;
	root["Alt"] = aMean;
	root["H"] = curAlt;
	root["Ref Alt"] = aRef;
	root["Max Alt"] = aMax;
  root["yaw"] = ypr[0] * 180/M_PI;
  root["pitch"] = ypr[1] * 180/M_PI;
  root["roll"] = ypr[2] * 180/M_PI;
  root["areal.x"] = aaReal.x;
  root["areal.y"] = aaReal.y;
  root["areal.z"] = aaReal.z;
  root["aworld.x"] = aaWorld.x;
  root["aworld.y"] = aaWorld.y;
  root["aworld.z"] = aaWorld.z;

	size_t written = root.printTo(charBuffer, 100);
	webSocket.broadcastTXT(charBuffer, written);*/
  webSocket.broadcastTXT("Hallo");
}

void notifyClient(const char msg[]) {

  Serial.println(msg);
  /*StaticJsonBuffer < 200 > jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["status"] = "event";
  root["message"] = msg;
  
  size_t written = root.printTo(charBuffer, 200);*/
  webSocket.broadcastTXT(msg);
}

void setupSockets() {
	// start webSocket server
 Serial.println("Setup Sockets");
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

  doBlink(2, 300);
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
  
  if( wifiStatus == WL_CONNECTED )
  {
    doBlink(3, 300);
    } else
    {
      doBlink(1, 300);
      }
	return wifiStatus == WL_CONNECTED;
}

boolean setupWiFiAP() {
  Serial.println("start wifi ap");
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
  Serial.println("done setup ap");
  wifiStatus = WL_CONNECTED;
  return true;
}

bool checkWiFi() {
	if (wifiStatus == WL_CONNECTED)
		return true;
	bool res = setupWiFiAP();
  Serial.println("Setup AP:  ");
  Serial.print(res);
  Serial.print("IP Addr:  ");
  Serial.println(WiFi.localIP());

  setupSockets();
  return res;
}


void doBlink(int count, int pause)
{
  for( int i=0; i<count; i++)
  {
    digitalWrite(LED_PIN, LOW);
    delay(pause);
    digitalWrite(LED_PIN, HIGH);
    delay(pause);
  }
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
}
