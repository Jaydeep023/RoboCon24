#include <ESP32Servo.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.


// define two Tasks for DigitalRead & AnalogRead
void Servo1( void *pvParameters );
void Servo2( void *pvParameters );
void Readdata( void *pvParameters );

bool ballflag = false;
bool siloflag = false;
Servo myservo0; 
Servo myservo180;  

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  myservo0.attach(15);
  myservo180.attach(18);
  

  // Now set up two Tasks to run independently.
  xTaskCreate(
    Servo1
    ,  "DigitalRead"  // A name just for humans
    ,  2048   // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

  xTaskCreate(
    Servo2
    ,  "AnalogRead" // A name just for humans
    ,  2048   // Stack size
    ,  NULL //Parameters for the task
    ,  1  // Priority
    ,  NULL ); //Task Handle
  xTaskCreate(
    Readdata
    ,  "AnalogRead" // A name just for humans
    ,  2048   // Stack size
    ,  NULL //Parameters for the task
    ,  1  // Priority
    ,  NULL ); //Task Handle

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Servo1(void *pvParameters)  // This is a Task.
{
  if(ballflag){
  for (int pos1 = 0; pos1 <= 180; pos1 += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo0.write(pos1);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}
  }
  if(siloflag){
  for (int pos1 = 180; pos1 >= 0; pos1 -= 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo0.write(pos1);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}
  }
}

void Servo2(void *pvParameters)  // This is a Task.
{
  if(ballflag){
	for (int pos2 = 180; pos2 >= 0; pos2 -= 1) { // goes from 180 degrees to 0 degrees
		myservo180.write(pos2);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	} 
  }
  if(siloflag){
	for (int pos2 = 0; pos2 <= 180; pos2 += 1) { // goes from 180 degrees to 0 degrees
		myservo180.write(pos2);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	} 
  }
}
void Readdata(void *pvParameters)  // This is a Task.
{
	while(1){

    if(data[0] == "a"){
      ballflag = true;
      siloflag = false;

    }
  } 
}