#include <Arduino_FreeRTOS.h>
#include <semphr.h>

const int OFF = 0;
const int BEAM = 1;
const int BALL = 2;

int mode = 0;
int refMode = 0;

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

TaskHandle_t xHandle;

void TaskRun( void *pvParameters );
void TaskReference( void *pvParameters );
void TaskOperationModeButtonRead( void *pvParameters );
void TaskReferenceModeButtonRead( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
      
  Serial.begin(9600);
  
  PI_init();

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

  xTaskCreate(
    
    TaskRun
    ,  (const portCHAR *) "Run"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  xHandle 
    
    );

  xTaskCreate(
    
    TaskReference
    ,  (const portCHAR *) "Run"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  1  // priority
    ,  xHandle 
    
    );

  xTaskCreate(
    
    TaskOperationModeButtonRead
    ,  (const portCHAR *) "Run"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  5  // priority
    ,  xHandle 
    
    );
    

    xTaskCreate(
    
    TaskReferenceModeButtonRead
    ,  (const portCHAR *) "Run"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  5  // priority
    ,  xHandle 
    
    );
    
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}


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

    if(digitalRead(1) == HIGH){
        mode = 0;
     }else if(digitalRead(1) == HIGH){
        mode = 1;
     }else if(digitalRead(1) == HIGH){
        mode = 2;
     }       
}

void TaskReferenceModeButtonRead(void *pvParameters) {
        
    (void) pvParameters;

    if(digitalRead(1) == HIGH){
        refMode = 0;
     }else if(digitalRead(1) == HIGH){
        refMode = 1;
     }else if(digitalRead(1) == HIGH){
        refMode = 2;
     }       
}

void TaskRun(void *pvParameters) {
        
  (void) pvParameters;

  long duration;
  long t = millis();

  for(;;){

     if(mode == 0){

        PI_reset();
        PID_reset();
      
     }else if(mode == 1){

        double ref_ang = referenceGetRef();
        double ang = analogRead(beam_pin); 
        double u_ang;

        taken_inner = true;
    
        while(taken_inner){
          
            if ( xSemaphoreTake( xSerialSemaphore_inner, ( TickType_t ) 5 ) == pdTRUE ){

                      // Calculate signal
                      double u_ang = limit(PI_calculateOutput(ang, ref_ang), uMin, uMax);
                      
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

            vTaskDelay(1);
            
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


