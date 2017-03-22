#include <Arduino_FreeRTOS.h>
#include <semphr.h>

int sensorValue = 500;

SemaphoreHandle_t xSerialSemaphore;

// define two tasks for Blink & AnalogRead
void TaskBlinkRed( void *pvParameters );
void TaskBlinkWhite( void *pvParameters );
void TaskAnalogRead( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlinkWhite
    ,  (const portCHAR *)"BlinkWhite"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // priority
    ,  NULL );
    
  xTaskCreate(
    TaskBlinkRed
    ,  (const portCHAR *)"BlinkRed"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  3  // priority
    ,  NULL );
    
  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}


int getSensorValue()
{
  return sensorValue;
  
  }

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlinkWhite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  
  for (;;)// A Task shall never return or exit.
  {
    int currentValue = 0;

    boolean taken = true;
    
    while(taken)  // A Task shall never return or exit.
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {

        taken = false;
        
        Serial.print("Setting white:");
        Serial.println(sensorValue);
  
        currentValue = sensorValue;
        vTaskDelay(100 / portTICK_PERIOD_MS); 
  
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
        vTaskDelay(1 /  portTICK_PERIOD_MS); 
    }
    
    Serial.println("white done");

    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( currentValue / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( currentValue / portTICK_PERIOD_MS ); // wait for one second
  }
}


void TaskBlinkRed(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(12, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {

    boolean taken = true;
    int currentValue = 0;
  
    while(taken)
    {
        
      // See if we can obtain or "Take" the Serial Semaphore.
      // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {

        taken = false;

        Serial.print("Setting red:");
        Serial.println(sensorValue);
                 
        currentValue = sensorValue;
        vTaskDelay(100/ portTICK_PERIOD_MS); 
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
      
        vTaskDelay(1 / portTICK_PERIOD_MS); 

    }

    Serial.println("red done");

    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( currentValue / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( currentValue / portTICK_PERIOD_MS ); // wait for one second
  }
}


void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  for (;;)
  {
    
    // read the input on analog pin 0:
    sensorValue = analogRead(A0);
    // print out the value you read:
   //Serial.println(sensorValue);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
