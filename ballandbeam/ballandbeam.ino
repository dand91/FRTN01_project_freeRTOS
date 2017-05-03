#include <Arduino_FreeRTOS.h>
#include <semphr.h>

const int OFF = 0;
const int BEAM = 1;
const int BALL = 2;

const int MANUAL = 0;
const int SQUARE = 1;
const int OPTIMAL = 2;

int mode = 1;
int refMode = 1;

double uMin = -10.0;
double uMax = 10.0;

long starttime;

double uff = 0.0;
double phiff = 0.0;

int beam_pin = 9;
int ball_pin = 10;

boolean taken_inner;
boolean taken_outer;

SemaphoreHandle_t xSerialSemaphore_inner;
SemaphoreHandle_t xSerialSemaphore_outer;
SemaphoreHandle_t xSerialSemaphore_ref;

TaskHandle_t xHandle;

void TaskRun( void *pvParameters );
void TaskReference( void *pvParameters );
void TaskOperationModeButtonRead( void *pvParameters );
void TaskReferenceModeButtonRead( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
      
  Serial.begin(9600);

  ref_init();
  PI_init();
  PID_init();

  // Inner semaphore
      
  if ( xSerialSemaphore_inner == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore_inner = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore_inner ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore_inner ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  
  // Outer semaphore
      
  if ( xSerialSemaphore_outer == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore_outer = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore_outer ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore_outer ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Reference semaphore
      
  if ( xSerialSemaphore_ref == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore_ref = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore_ref ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore_ref ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }


  Serial.println("Semaphores initiated");

  xTaskCreate(
    
    TaskRun
    ,  (const portCHAR *) "Run"
    ,  256 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL 
    
    );

  xTaskCreate(
    
    TaskReference
    ,  (const portCHAR *) "Reference"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL 
    
    );

  xTaskCreate(
    
    TaskOperationModeButtonRead
    ,  (const portCHAR *) "OperationsButton"
    ,  64 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL 
    
    );
    

    xTaskCreate(
    
    TaskReferenceModeButtonRead
    ,  (const portCHAR *) "ReferenceButton"
    ,  64 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  NULL 
    
    );
    
//     vTaskStartScheduler();
     Serial.println("Tasks initiated");

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){}

float limit(float u, float umin, float umax) {

  if (u < umin) {
  
      u = umin;
      
  } else if (u > umax) {
      
      u = umax;
      
  }
  
  return u;

}

void TaskOperationModeButtonRead(void *pvParameters) {
    
    (void) pvParameters;

     vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 1);

  for(;;){

    if(digitalRead(1) == HIGH){
        mode = OFF;
     }else if(digitalRead(2) == HIGH){
        mode = BEAM;
     }else if(digitalRead(3) == HIGH){
        mode = BALL;
     }       

     vTaskDelay(10 / portTICK_PERIOD_MS); 

  }
}

void TaskReferenceModeButtonRead(void *pvParameters) {
        
    (void) pvParameters;

     vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 1);

  for(;;){

    if(digitalRead(4) == HIGH){
        refMode = MANUAL;
     }else if(digitalRead(5) == HIGH){
        refMode = SQUARE;
     }else if(digitalRead(6) == HIGH){
        refMode = OPTIMAL;
     }       

     vTaskDelay(10 / portTICK_PERIOD_MS); 

  }
}

void TaskRun(void *pvParameters) {

  (void) pvParameters;

  vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 3);

  Serial.print("Running");

  long duration;
  long t = millis();

  for(;;){

     if(mode == 0){

        PI_reset();
        PID_reset();
      
     }else if(mode == 1){

        double ref_ang = referenceGetRef();
             
        double ang = analogRead(beam_pin); 

        Serial.print(" ref ");
        Serial.print(ref_ang);
        Serial.print(" ang ");
        Serial.print(ang);
        Serial.println();

        double u_ang;

        taken_inner = true;
    
        while(taken_inner){
          
            if ( xSemaphoreTake( xSerialSemaphore_inner, ( TickType_t ) 5 ) == pdTRUE ){

                      // Calculate signal

                      Serial.print(" result ");
                      Serial.print(PI_calculateOutput(ang, ref_ang));
                      double u_ang = limit(PI_calculateOutput(ang, ref_ang), uMin, uMax);

                      Serial.print(" u_ang ");
                      Serial.print(u_ang);
                       
                      // Set output
                      analogWrite(beam_pin,u_ang);
                      
                      // Update state
                      PI_updateState(u_ang);
    
                      xSemaphoreGive( xSerialSemaphore_inner ); // Now free or "Give" the Serial Port for others.
                      taken_inner = false;
          }else{
            
              vTaskDelay(1 / portTICK_PERIOD_MS);
            
           }    
        }
      
     }else if(mode == 2){

        double pos = analogRead(ball_pin);
        double ang = analogRead(beam_pin);
        double ref_pos = referenceGetRef();
        double ang_ref;
        double u;
        
        phiff = referenceGetPhiff();
        uff = referenceGetUff();
        
        taken_outer = true;
    
        while(taken_outer){
          
            if ( xSemaphoreTake( xSerialSemaphore_outer, ( TickType_t ) 5 ) == pdTRUE ){

                    ang_ref = limit(PI_calculateOutput(pos, ref_pos) + phiff, uMin, uMax);

                    while(taken_inner){
            
                        if ( xSemaphoreTake( xSerialSemaphore_inner, ( TickType_t ) 5 ) == pdTRUE ){
        
                            double u_ang = limit(PI_calculateOutput(ang, ang_ref) + uff, uMin, uMax);
                            
                            analogWrite(beam_pin,u_ang);
                            
                            PI_updateState(u_ang - uff);
          
                            xSemaphoreGive( xSerialSemaphore_inner ); // Now free or "Give" the Serial Port for others.
                            taken_inner = false;
                                        
                        }else{
                
                          vTaskDelay(1 / portTICK_PERIOD_MS);
                          
                        }
                    }

                    PID_updateState(ang_ref - phiff);
                    xSemaphoreGive( xSerialSemaphore_outer); // Now free or "Give" the Serial Port for others.
                    taken_outer = false;
                
          }else{

            vTaskDelay(1 / portTICK_PERIOD_MS);
            
          }
        }
     } 
     
      t = t + PI_getHMillis();
      
      duration = t - millis();
      
      if (duration > 0) {
         
          vTaskDelay(duration / portTICK_PERIOD_MS); 

      }
    }
  }



