#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>  // Include SoftwareSerial for Bluetooth communication

// MPU6050 setup
Adafruit_MPU6050 mpu;
#define PITCH_MIN_THRESHOLD -15.0  // Minimum pitch threshold
#define PITCH_MAX_THRESHOLD 15.0   // Maximum pitch threshold

// Ultrasonic sensor setup
const int trigPin = 9;            // Pin for ultrasonic trigger
const int echoPin = 10;           // Pin for ultrasonic echo
#define ULTRASONIC_MIN_THRESHOLD 5   // Minimum distance threshold in cm
#define ULTRASONIC_MAX_THRESHOLD 9   // Maximum distance threshold in cm

// LED pins
#define MPU_LED_PIN 8             // LED for MPU threshold
#define ULTRASONIC_LED_PIN 12     // LED for ultrasonic threshold
#define WEIGHT_LED_PIN 13         // LED for weight threshold

// Bluetooth module setup using Hardware Serial (pins 1 and 0)
#define BT_SERIAL Serial  // Use hardware serial for Bluetooth communication

// HX711 weight sensor setup
#define ADDO  3    // Data Out
#define ADSK  2    // SCK
unsigned long convert; // Variable for weight data
#define WEIGHT_THRESHOLD 13000  // Weight threshold in grams

// Function to read from HX711
unsigned long ReadCount() {
    unsigned long Count = 0;
    unsigned char i;

    digitalWrite(ADSK, LOW);

    while (digitalRead(ADDO)); // Wait for the data to be available

    for (i = 0; i < 24; i++) {
        digitalWrite(ADSK, HIGH);
        Count = Count << 1;
        digitalWrite(ADSK, LOW);
        if (digitalRead(ADDO)) Count++;
    }

    digitalWrite(ADSK, HIGH);
    Count = Count ^ 0x800000;  // Two's complement
    digitalWrite(ADSK, LOW);

    return (Count);  // Return the weight value
}

void setup() {
    // Start Bluetooth serial
    BT_SERIAL.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        BT_SERIAL.println("Failed to find MPU6050 chip");
        while (1);
    }

    // PLX-DAQ Initialization
    BT_SERIAL.println("Pitch,Distance,Weight,MPU_LED,Ultrasonic_LED,Weight_LED");

    // Initialize LED pins
    pinMode(MPU_LED_PIN, OUTPUT);
    digitalWrite(MPU_LED_PIN, LOW);
    pinMode(ULTRASONIC_LED_PIN, OUTPUT);
    digitalWrite(ULTRASONIC_LED_PIN, LOW);
    pinMode(WEIGHT_LED_PIN, OUTPUT);
    digitalWrite(WEIGHT_LED_PIN, LOW);

    // Initialize ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Initialize HX711 sensor pins
    pinMode(ADDO, INPUT_PULLUP); // Set Data Out pin to input
    pinMode(ADSK, OUTPUT);       // Set Clock pin to output
}

void loop() {
    // --------------------------
    // MPU6050 Sensor Readings
    // --------------------------
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate pitch angle
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;

    // Check if pitch threshold is exceeded
    bool pitchExceeded = (pitch < PITCH_MIN_THRESHOLD || pitch > PITCH_MAX_THRESHOLD);
    digitalWrite(MPU_LED_PIN, pitchExceeded ? HIGH : LOW);

    // --------------------------
    // Ultrasonic Sensor Readings
    // --------------------------
    long duration;
    int distanceCm;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the echo pulse
    duration = pulseIn(echoPin, HIGH);

    // Calculate distance in cm
    distanceCm = duration * 0.034 / 2;

    // Check if ultrasonic threshold is exceeded
    bool ultrasonicExceeded = (distanceCm < ULTRASONIC_MIN_THRESHOLD || distanceCm > ULTRASONIC_MAX_THRESHOLD);
    digitalWrite(ULTRASONIC_LED_PIN, ultrasonicExceeded ? HIGH : LOW);

    // --------------------------
    // Read weight data from HX711
    // --------------------------
    convert = ReadCount();  // Get the weight value

    // Check if weight threshold is exceeded
    bool weightExceeded = (convert > WEIGHT_THRESHOLD);
    digitalWrite(WEIGHT_LED_PIN, weightExceeded ? HIGH : LOW);

    // --------------------------
    // Send Data to PLX-DAQ via Bluetooth
    // --------------------------
    BT_SERIAL.print(pitch, 2);   // Pitch angle (2 decimal places)
    BT_SERIAL.print(",");
    BT_SERIAL.print(distanceCm); // Distance in cm
    BT_SERIAL.print(",");
    BT_SERIAL.print(convert);    // Weight value (no units)
    BT_SERIAL.print(",");
    BT_SERIAL.print(pitchExceeded ? "ON" : "OFF"); // MPU LED state
    BT_SERIAL.print(",");
    BT_SERIAL.print(ultrasonicExceeded ? "ON" : "OFF"); // Ultrasonic LED state
    BT_SERIAL.print(",");
    BT_SERIAL.println(weightExceeded ? "ON" : "OFF"); // Weight LED state

    // Add a short delay
    delay(30000);
}
