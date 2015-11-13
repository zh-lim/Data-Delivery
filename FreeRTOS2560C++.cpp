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

#define SONAR_COUNT					3
#define SONAR_LEFT_ID				0
#define SONAR_RIGHT_ID				1
#define SONAR_GLOVE_ID				2

// How often to read each sonar in Hz (Take note : EZ2 and EZ4 sonar maximum frequency is 20 Hz)
#define SONAR_READING_FREQUENCY		15
#define SONAR_READING_PERIOD_MS		1000 / SONAR_READING_FREQUENCY
#define SONAR_WAIT_PERIOD_MS		10

#define RPI_SERIAL_PORT_BAUDRATE	115200

// ADC-Parameters
#define MICROVOLTS_PER_CM			3900
#define MICROVOLTS_PER_UNIT			4900
//#define DISTANCE_SCALE_FACTOR		MICROVOLTS_PER_UNIT / MICROVOLTS_PER_CM
#define DISTANCE_SCALE_FACTOR		13

// ADC channel to Sonar Mappings
#define SONAR_LEFT_TRIGGER			34
#define SONAR_LEFT_ECHO				35

// Sonar right pins
#define SONAR_RIGHT_TRIGGER			32
#define SONAR_RIGHT_ECHO			33

// Sonar glove pins
#define SONAR_GLOVE_TRIGGER			48
#define SONAR_GLOVE_ECHO			49

// Motor Pin
#define MOTOR_PIN_RIGHT				8
#define MOTOR_PIN_LEFT				9
#define MOTOR_PIN_GLOVE				7

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
TickType_t lastTickValue1;
TickType_t lastTickValue2;
TickType_t lastTickValue3;
// Averaging filter settings
#define AVG_FILTER_SAMPLE_SIZE		3
#define DEFAULT_BUFFER_SIZE			20

// accelerometer
#define SENSOR_GRAVITY_EARTH		9.80665F
#define lsm303accel_mg_lsb			0.001F

// Semaphore for maintaining mutual exclusion of ultrasonic transmission
SemaphoreHandle_t SEMA_SONAR;

SemaphoreHandle_t SEMA_COMPASS;

// Queue for containing sonar readings
QueueHandle_t ARR_SONAR_QUEUE[SONAR_COUNT];
QueueHandle_t compassQueue;
QueueHandle_t acclerationQueue;

// Stores the last read value from each sonar
volatile int ARR_SONAR_SAMPLES[SONAR_COUNT][AVG_FILTER_SAMPLE_SIZE] = {{0},{0}};
volatile int ARR_SONAR_READ_COUNT[SONAR_COUNT] = {0};

// Temporary buffers for serial despatcher
char itemBuffer1[DEFAULT_BUFFER_SIZE] = {0};
char itemBuffer2[DEFAULT_BUFFER_SIZE] = {0};
char itemBuffer3[18] = {0};
SensorReading srLeft;
SensorReading srRight;
//SensorReading srChest;
SensorReading srGlove;
SensorReading compassReadingData;
SensorReading accReadingData;


//accelerometer
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
LSM303::vector<int16_t> a_running_min = {32767, 32767, 32767}, a_running_max = {-32768, -32768, -32768};

char report[16] = {0};
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

unsigned long sonarReading1;
unsigned long sonarReading2;
unsigned long sonarReading3;

unsigned long distance3;
unsigned long distance1;
unsigned long distance2;

int gloveCount = 0;
int rightCount = 0;
int leftCount = 0;
int sonarTrigDelay = 0;

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
	pinMode(SONAR_RIGHT_ECHO, INPUT);
	pinMode(SONAR_GLOVE_ECHO, INPUT);

	pinMode(SONAR_LEFT_TRIGGER, OUTPUT);
	pinMode(SONAR_RIGHT_TRIGGER, OUTPUT);
	pinMode(SONAR_GLOVE_TRIGGER, OUTPUT);		

	// Set Motor pin to output and Low
	pinMode(MOTOR_PIN_LEFT, OUTPUT);
	digitalWrite(MOTOR_PIN_LEFT, LOW);
	pinMode(MOTOR_PIN_RIGHT, OUTPUT);
	digitalWrite(MOTOR_PIN_RIGHT, LOW);
	pinMode(MOTOR_PIN_GLOVE, OUTPUT);
	digitalWrite(MOTOR_PIN_GLOVE, LOW);
		
	// Make all sonar enable lines low to disable all sonars
	digitalWrite(SONAR_LEFT_TRIGGER, LOW);
	digitalWrite(SONAR_RIGHT_TRIGGER, LOW);
	digitalWrite(SONAR_GLOVE_TRIGGER, LOW);
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

void readAndEnqueueSonarReading(uint8_t sonarId,SensorReading * sr)
{
	
	//// Maintain exclusivity of sonar enable lines - only 1 sonar at a time may be active
	xSemaphoreTake(SEMA_SONAR, portMAX_DELAY);
	if(sonarId == 0){
		vTaskSuspendAll ();
		lastTickValue2 = xTaskGetTickCount();
		digitalWrite(SONAR_LEFT_TRIGGER, HIGH);
		vTaskDelayUntil(&lastTickValue2, SONAR_WAIT_PERIOD_MS_TICK_INCREMENT);
		digitalWrite(SONAR_LEFT_TRIGGER, LOW);
		sonarReading2 = pulseIn(SONAR_LEFT_ECHO, HIGH,5000);
		
		//Serial.println("Left Sonar Reading:");
		sr->name = 's';
		//convertToDecimalString((char *)sr->data, sonarReading2);
		distance2 = sonarReading2/58; // converts sonar reading to centimeters
		//
		//Serial.println("LEFT:");
		//Serial.println(distance2);
		
		if(distance2 <=70 && distance2 > 0)
		{
			leftCount++;
		}
		else if(distance2 > 70)
		{
			leftCount= 0;
		}
		
		if(leftCount > 25)
		{
			digitalWrite(MOTOR_PIN_LEFT, HIGH);
			//sonarTrigDelay++;
		}
		else
		{
			digitalWrite(MOTOR_PIN_LEFT, LOW);
			
		}
		xTaskResumeAll ();
	}
	
	else if(sonarId == 1){
		vTaskSuspendAll ();
		lastTickValue = xTaskGetTickCount();
		digitalWrite(SONAR_RIGHT_TRIGGER, HIGH);
		vTaskDelayUntil(&lastTickValue, SONAR_WAIT_PERIOD_MS_TICK_INCREMENT);
		digitalWrite(SONAR_RIGHT_TRIGGER, LOW);
		sonarReading1 = pulseIn(SONAR_RIGHT_ECHO, HIGH,5000);
		
		//Serial.println("Right Sonar Reading:");
		//sr->name = 's';
		//convertToDecimalString((char *)sr->data, sonarReading1);
		distance1 = sonarReading1/58; // converts sonar reading to centimeters
		//Serial.println("RIGHT:");
		//Serial.println(distance1);
		if(distance1 <=70 && distance1 > 0)
		{
			rightCount++;
		}
		else if(distance1 > 70)
		{
			rightCount= 0;
		}
		
		if(rightCount > 25)
		{
			digitalWrite(MOTOR_PIN_RIGHT, HIGH);
			//sonarTrigDelay++;
		}
		else
		{
			digitalWrite(MOTOR_PIN_RIGHT, LOW);
			
		}
		xTaskResumeAll();
	}
	else if(sonarId == 2){
		vTaskSuspendAll ();
		lastTickValue3 = xTaskGetTickCount();
		digitalWrite(SONAR_GLOVE_TRIGGER, HIGH);
		vTaskDelayUntil(&lastTickValue3, SONAR_WAIT_PERIOD_MS_TICK_INCREMENT);
		digitalWrite(SONAR_GLOVE_TRIGGER, LOW);
		sonarReading3 = pulseIn(SONAR_GLOVE_ECHO, HIGH,5000);
		distance3 = sonarReading3/58; // converts sonar reading to centimeters
		//Serial.println(sr->data);
		
		if(distance3 <= 80 && distance3 > 0)
		{
			gloveCount++;
		}
		else
		{		
			gloveCount= 0;
		}
		
		if(gloveCount > 15){
			digitalWrite(MOTOR_PIN_GLOVE, HIGH);
			//sonarTrigDelay++;
		}else
		{
			digitalWrite(MOTOR_PIN_GLOVE, LOW);
			
		}
		
		xTaskResumeAll ();
	}
	xSemaphoreGive(SEMA_SONAR);
}

void serialDespatcher(void * args)
{
	BaseType_t q1Result;
	
	while(true)
	{
		//// Read sonar1 and sonar2 data, don't wait for queue if they are empty
		//q1Result = xQueueReceive(ARR_SONAR_QUEUE[0], itemBuffer1, 0);
		//if(q1Result == pdTRUE)
		//{
			//// Send sonar data to RPi
			//
			//SPROTSend((byte_t *)itemBuffer1, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//
		//}
		//
		////Send acceleration data
		//if(xQueueReceive(compassQueue, itemBuffer2, 0) == pdTRUE)
		//{
			//SPROTSend((byte_t *)itemBuffer2, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			////Serial.println(itemBuffer1);
		//}
		
		//Send compass data
		if(xQueueReceive(acclerationQueue, itemBuffer3, 0) == pdTRUE)
		{		
			SPROTSend((byte_t *)itemBuffer3, 0, QUEUE_ITEM_SIZE, SPROT_SEND_TIMEOUT);
			//Serial.println(itemBuffer3);
		}
	}
	//vTaskDelay(100);
}

void sonarLeft(void *p)
{
	while(1)
	{
		readAndEnqueueSonarReading(SONAR_LEFT_ID,&srLeft);
		vTaskDelay(SONAR_READING_PERIOD_MS);
	}
}

void sonarRight(void *p)
{
	while(1)
	{
		readAndEnqueueSonarReading(SONAR_RIGHT_ID,&srRight);
		vTaskDelay(SONAR_READING_PERIOD_MS);
	}
}

void sonarGlove(void *p)
{
	while(1)
	{
		readAndEnqueueSonarReading(SONAR_GLOVE_ID,&srGlove);
		vTaskDelay(SONAR_READING_PERIOD_MS);
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
		snprintf(report, sizeof(report),"%d,%d,%d,%d",
		heading,compass.a.x,compass.a.y,compass.a.z);
		//snprintf(report, sizeof(report),"%d,%d,%d,%d,",
		//300,1,0,0);
		memcpy(accReadingData.data, report, 10);
		xQueueReset(acclerationQueue);
		xQueueOverwrite(acclerationQueue,&accReadingData);
		//Serial.println(report);
		//if(sonarTrigDelay>9){
			//sonarTrigDelay = 0;
			//vTaskDelay(1);
		//}
	}
}

void barometerReading(void *p){
	while(1){
		float pressure = barometer.readPressureMillibars();
		float altitude = barometer.pressureToAltitudeMeters(pressure);
		float temperature = barometer.readTemperatureC();
		
		dtostrf(pressure, 2, 2, pressureString);
		dtostrf(altitude, 2, 2, altString);
		dtostrf(temperature, 2, 2, tempString);
		snprintf(baroReport, sizeof(baroReport),"p: %s a: %s t: %s",
		pressureString, altString, tempString);
		Serial.println(baroReport);	
	}
}

void gyroReading(void *p){
	while (1)
	{
		gyro.read();
		snprintf(gyroReport, sizeof(gyroReport), "%d, %d, %d", 
		gyro.g.x, gyro.g.y, gyro.g.z);
		Serial.println(gyroReport);
		vTaskDelay(500);
	}
}

void compassReading(void *p) {
	while(1)
	{
		compass.readMag();
		compassReadingData.name = 'C';
		//float heading = compass.heading();
		//snprintf(compassReport, sizeof(compassReport),"%2d",
		//heading);
		snprintf(compassReport, sizeof(compassReport),"%2d",
		compass.m.x);
		memcpy(compassReadingData.data, compassReport, 16);
		xQueueOverwrite(compassQueue,&compassReadingData);	
		Serial.println(compassReport);
		//delay(10);
		vTaskDelay(10);
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
	xTaskCreate(sonarGlove, "snglv", STACK_SIZE, NULL, TASK_PRIORITY + 1, NULL);
	xTaskCreate(sonarLeft, "snlft", STACK_SIZE, NULL, TASK_PRIORITY + 1, NULL);
	xTaskCreate(sonarRight, "snrgt", STACK_SIZE, NULL, TASK_PRIORITY + 1, NULL);
	xTaskCreate(accReading, "accrd", STACK_SIZE, NULL, TASK_PRIORITY +1, NULL);
	//xTaskCreate(barometerReading, "brmt", STACK_SIZE, NULL, TASK_PRIORITY +1, NULL);
	//xTaskCreate(gyroReading, "gyro", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(compassReading, "cprd", STACK_SIZE, NULL, TASK_PRIORITY, NULL);
	xTaskCreate(serialDespatcher, "srdsp", STACK_SIZE, NULL, TASK_PRIORITY+1, NULL);
	//xTaskCreate(calibrate, "cali", STACK_SIZE, NULL, TASK_PRIORITY, NULL);

	vTaskStartScheduler();
	return 0;
}