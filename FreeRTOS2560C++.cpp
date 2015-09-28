/*
 * FreeRTOS2560C__.cpp
 *
 * Created: 8/27/2015 11:30:20 AM
 *  Author: Chia Sheng
 */ 


#include <avr/io.h>
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

//arduino
#include <Arduino.h>
#include <Wire.h>
#include "HardwareSerial.h"
#include "LSM303.h"
#include "LPS.h" 
#include "L3G.h"

//protocol
#include "sprotapi.h"
#include "SensorReading.h"
#include "myutil.h"

#define SONAR_COUNT					2
#define SONAR_LEFT_ID				0
#define SONAR_RIGHT_ID				1

// How often to read each sonar in Hz (Take note : EZ2 and EZ4 sonar maximum frequency is 20 Hz)
#define SONAR_READING_FREQUENCY		2
#define SONAR_READING_PERIOD_MS		1000 / SONAR_READING_FREQUENCY
#define SONAR_WAIT_PERIOD_MS		10

#define MOTOR_PIN					49

#define RPI_SERIAL_PORT_BAUDRATE		115200

// ADC-Parameters
#define MICROVOLTS_PER_CM			3900
#define MICROVOLTS_PER_UNIT			4900
//#define DISTANCE_SCALE_FACTOR		MICROVOLTS_PER_UNIT / MICROVOLTS_PER_CM
#define DISTANCE_SCALE_FACTOR		13

// ADC channel to Sonar Mappings
#define SONAR_LEFT_TRIGGER			30
#define SONAR_LEFT_ECHO				31
#define SONAR_LEFT_ADC_CHANNEL		A0
#define SONAR_RIGHT_ADC_CHANNEL		A7
#define SONAR3_ADC_CHANNEL			A13

// Sonar output enable pins
#define SONAR_LEFT_ENABLE_PIN		13
#define SONAR_RIGHT_ENABLE_PIN		12
#define SONAR3_ENABLE_PIN			11

// Sonar power pins
#define SONAR_LEFT_POWER_PIN		7
#define SONAR_RIGHT_POWER_PIN		6

// Task parameters
#define STACK_SIZE					150
#define TASK_PRIORITY				tskIDLE_PRIORITY + 1

// Queue parameters
#define QUEUE_LENGTH				1
#define QUEUE_ITEM_SIZE				sizeof(SensorReading)
#define QUEUE_WAIT_TIME				portMAX_DELAY

// For delay functions
const TickType_t SONAR_WAIT_PERIOD_MS_TICK_INCREMENT = SONAR_WAIT_PERIOD_MS;
TickType_t lastTickValue;

// Averaging filter settings
#define AVG_FILTER_SAMPLE_SIZE		3
#define DEFAULT_BUFFER_SIZE			20

// accelerometer
#define SENSOR_GRAVITY_EARTH	9.80665F
#define lsm303accel_mg_lsb		0.001F

// Semaphore for maintaining mutual exclusion of ultrasonic transmission
SemaphoreHandle_t SEMA_SONAR;

SemaphoreHandle_t SEMA_COMPASS;

// Queue for containing sonar readings
QueueHandle_t ARR_SONAR_QUEUE[SONAR_COUNT];
QueueHandle_t IMUQueue;
QueueHandle_t compassQueue;
QueueHandle_t acclerationQueue;
QueueHandle_t barometerQueue;
QueueHandle_t gyroscopeQueue;

// Stores the last read value from each sonar
volatile int ARR_SONAR_SAMPLES[SONAR_COUNT][AVG_FILTER_SAMPLE_SIZE] = {{0},{0}};
volatile int ARR_SONAR_READ_COUNT[SONAR_COUNT] = {0};

// Temporary buffers for serial despatcher
char itemBuffer1[DEFAULT_BUFFER_SIZE] = {0};
char itemBuffer2[DEFAULT_BUFFER_SIZE] = {0};
char itemBuffer3[DEFAULT_BUFFER_SIZE] = {0};
SensorReading srLeft;
SensorReading IMUdata;
SensorReading compassReadingData;
SensorReading accReadingData;
SensorReading baroReadingData;
SensorReading gyroReadingData;

// array to store temporary data for sonar
int sonardata [10];
int count = 0;

//accelerometer
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
LSM303::vector<int16_t> a_running_min = {32767, 32767, 32767}, a_running_max = {-32768, -32768, -32768};

char report[16];
char compassReport[16];

//barometer
LPS barometer;
char baroReport[32];
char pressureString[8];
char altString[8];
char tempString[8];

//gyroscope
L3G gyro;
char gyroReport[16];


void setup(void)
{
	// Arduino initialization for some common functions
	init();
	
	//wire initialization
	Wire.begin();
	//compass initialization
 	compass.init();
 	compass.enableDefault();
 	compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
	compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
		
	barometer.init();
	barometer.enableDefault();
	
	gyro.init();
	gyro.enableDefault();
	// Select the voltage source to be used as the ADC reference
	analogReference(DEFAULT);
	
	// Set analog pins to input mode
	pinMode(SONAR_LEFT_ECHO, INPUT);
	
	pinMode(SONAR_LEFT_ADC_CHANNEL, INPUT);
	
	pinMode(SONAR_LEFT_TRIGGER, OUTPUT);
	
	// Set sonar enable pins to output
	pinMode(SONAR_LEFT_ENABLE_PIN, OUTPUT);
	
	// Set all power outputs to output and HIGH
	pinMode(SONAR_LEFT_POWER_PIN, OUTPUT);
	
	// Make all sonar enable lines low to disable all sonars
	digitalWrite(SONAR_LEFT_TRIGGER, LOW);
	
	// motor pin output
	pinMode(MOTOR_PIN, OUTPUT);
	
	SPROTInit(115200.0);
	
	// Create a queue for each sonar
	for(int i=0; i<SONAR_COUNT; i++)
	{
		ARR_SONAR_QUEUE[i] = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	}
	
	// Create a queue for compass readings
	acclerationQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	compassQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
	SEMA_SONAR = xSemaphoreCreateBinary();
	xSemaphoreGive(SEMA_SONAR);
	
}

void readAndEnqueueSonarReading(uint8_t sonarId,SensorReading * sr,uint8_t sonarADCChannel, uint8_t sonarEnablePin)
{

	int sonarReading;
	int distance;
	//// Maintain exclusivity of sonar enable lines - only 1 sonar at a time may be active
	xSemaphoreTake(SEMA_SONAR, portMAX_DELAY);
	digitalWrite(SONAR_LEFT_TRIGGER, HIGH);
	lastTickValue = xTaskGetTickCount();
	vTaskDelayUntil(&lastTickValue, SONAR_WAIT_PERIOD_MS_TICK_INCREMENT);
	// Read sonar from corresponding ADC channel, perform value adjustments as needed
	//delayMicroseconds(10);
	digitalWrite(SONAR_LEFT_TRIGGER, LOW);
	sonarReading = pulseIn(SONAR_LEFT_ECHO, HIGH);
	if(count < 9){
		sonardata[count] = sonarReading;
		count++;
	}else if(count == 9){
		count = 0;
		distance = (sonarFilter(sonardata)/2910);
		if(distance < 1.0)
			digitalWrite(MOTOR_PIN, HIGH);
		else
			digitalWrite(MOTOR_PIN, LOW);
	}                           
	//sonarReading = (analogRead(sonarADCChannel) * DISTANCE_SCALE_FACTOR) / 10;
	xSemaphoreGive(SEMA_SONAR);
	sr->name = 's';
	convertToDecimalString((char *)sr->data, sonarReading);
	//Serial.println(sr->data);
	// Enqueue sonar reading for transmission over serial line
	//xQueueOverwrite(ARR_SONAR_QUEUE[0], sr);
}

void serialDespatcher(void * args)
{
	BaseType_t q1Result;
	
	while(true)
	{
		// Read sonar1 and sonar2 data, don't wait for queue if they are empty
		q1Result = xQueueReceive(ARR_SONAR_QUEUE[0], itemBuffer1, 0);
		if(q1Result == pdTRUE)
		{
			// Send sonar data to RPi
			
			SPROTSend((byte_t *)itemBuffer1, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			
		}
		
		//Send acceleration data
		if(xQueueReceive(compassQueue, itemBuffer2, 0) == pdTRUE)
		{
			SPROTSend((byte_t *)itemBuffer2, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//Serial.println(itemBuffer1);
		}
		
		//Send compass data
		if(xQueueReceive(acclerationQueue, itemBuffer3, 0) == pdTRUE)
		{
			SPROTSend((byte_t *)itemBuffer3, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//Serial.println(itemBuffer1);
		}
	}
}

void sonarLeft(void *p)
{
	while(1)
	{
		
		readAndEnqueueSonarReading(SONAR_LEFT_ID,&srLeft,SONAR_LEFT_ADC_CHANNEL, SONAR_LEFT_ENABLE_PIN);
		//vTaskDelay(SONAR_READING_PERIOD_MS);
	}
}

#define STACK_DEPTH 64

void vApplicationIdleHook()
{
	//	Do	nothing.
}

void accReading(void *p) {
	while(1)
	{
		compass.read();
		accReadingData.name = 'a';
		int heading = compass.heading();
		snprintf(report, sizeof(report),"%d,%d,%d,%d,",
		heading,compass.a.x,compass.a.y,compass.a.z);
		memcpy(accReadingData.data, report, 16);
		xQueueOverwrite(acclerationQueue,&accReadingData);	
		Serial.println(report);	
		
	}
}

void barometerReading(void *p){
	while(1){
		float pressure = barometer.readPressureMillibars();
		float altitude = barometer.pressureToAltitudeMeters(pressure);
		float temperature = barometer.readTemperatureC();
		
		//dtostrf(pressure, 2, 2, pressureString);
		dtostrf(altitude, 2, 2, altString);
		//dtostrf(temperature, 2, 2, tempString);
		snprintf(baroReport, sizeof(baroReport),"a: %s",
		altString);
		//Serial.println(baroReport);	
	}
}

void gyroReading(void *p){
	while (1)
	{
		gyro.read();
		snprintf(gyroReport, sizeof(gyroReport), "%d, %d, %d", 
		gyro.g.x, gyro.g.y, gyro.g.z);
		//Serial.println(gyroReport);
		vTaskDelay(500);
	}
}

void compassReading(void *p) {
	while(1)
	{
		compass.readMag();
		compassReadingData.name = 'C';
		float heading = compass.heading();
		//snprintf(compassReport, sizeof(compassReport),"%2d",
		//heading);
		snprintf(compassReport, sizeof(compassReport),"%2d",
		compass.m.x);
		memcpy(compassReadingData.data, compassReport, 16);
		xQueueOverwrite(compassQueue,&compassReadingData);	
		//Serial.println(compassReport);
		//delay(10);
		vTaskDelay(10);
	}
}

void IMUreading(void *p)
{
	while(1)
	{
		// accelerator reading
		compass.read();
		gyro.read();
		accReadingData.name = 'a';
		int heading = compass.heading();
		int altitude = barometer.pressureToAltitudeMeters(barometer.readPressureMillibars());
		snprintf(report, sizeof(report),"%d,%d,%d,%d,%d,%d,%d,%d,",
		heading,compass.a.x,compass.a.y,compass.a.z,altitude,gyro.g.x,gyro.g.y,gyro.g.z);
		memcpy(IMUdata.data, report, 16);
		//memcpy(accReadingData.data, report, 16);
		xQueueOverwrite(acclerationQueue,&accReadingData);
	}
}

void calibrate(void *p)
{
	while(1)
	{
		compass.read();
		running_min.x = min(running_min.x, compass.m.x);
		running_min.y = min(running_min.y, compass.m.y);
		running_min.z = min(running_min.z, compass.m.z);
		running_max.x = max(running_max.x, compass.m.x);
		running_max.y = max(running_max.y, compass.m.y);
		running_max.z = max(running_max.z, compass.m.z);
		
		a_running_min.x = min(a_running_min.x, compass.a.x);
		a_running_min.y = min(a_running_min.y, compass.a.y);
		a_running_min.z = min(a_running_min.z, compass.a.z);
		a_running_max.x = max(a_running_max.x, compass.a.x);
		a_running_max.y = max(a_running_max.y, compass.a.y);
		a_running_max.z = max(a_running_max.z, compass.a.z);
		
		compass.m_min = running_min;
		compass.m_max = running_max;
		
		compass.a_max = a_running_max;
		compass.a_min = a_running_min;
		
		//Serial.println(compass.m_min.x);
	}
}

int main(void)
{	
	setup();
	
	xTaskCreate(sonarLeft, "snlft", STACK_SIZE, NULL, TASK_PRIORITY + 1, NULL);
	//xTaskCreate(serialDespatcher, "srdsp", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(calibrate, "cali", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(IMUreading, "imur", STACK_SIZE, NULL, TASK_PRIORITY, NULL);

// readings from the IMU will be executed in 1 task 
	//xTaskCreate(accReading, "accrd", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(barometerReading, "brmt", STACK_SIZE, NULL, TASK_PRIORITY +1, NULL);
	//xTaskCreate(gyroReading, "gyro", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(compassReading, "cprd", STACK_SIZE, NULL, TASK_PRIORITY, NULL);

	vTaskStartScheduler();
	return 0;
}