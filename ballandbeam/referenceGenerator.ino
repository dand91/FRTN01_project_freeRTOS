const int MANUAL = 0;
const int SQUARE = 1;
const int OPTIMAL = 2;

const double K_PHI = 4.5;
const double K_V = 10.0;

double amplitude;
double period;
double max_ctrl;
double sign = -1.0;
double ref;
double manual;

boolean ampChanged;
boolean periodChanged;
boolean maxCtrlChanged;
boolean parChanged = false;

void ref_init(){

  amplitude = 5.0;
  period = 15.0;
  manual = 0.0;
  ref = 0.0;
  max_ctrl = 0.1;
  refMode = MANUAL;
  
  }

void TaskReference(void *pvParameters) {
        
    (void) pvParameters;

    double h = 0;
    double timebase = millis();
    double timeleft = 0;
    long ref_duration;
    
    double setpoint = 0.0;
    double new_setpoint;
    double u0 = 0.0;
    double distance = 0.0;
    double now = 0.0;
    double t = 0.0;
    double ref_t = 0.0;
    double tf = 0.001 * timebase;
    double ts = tf;
    double T = 0.0;
    double zf = 0.0;
    double z0 = 0.0;  


    for(;;){

      if(mode == MANUAL){
        
        setpoint = manual;
        ref = manual;
        
      }else{

        timeleft -= h;

        if(getParChanged()){
          
          timeleft = 0;
              
        }
        if(timeleft <= 0){

          timeleft += (long)(500.0 * period); 
          
        }
        
        new_setpoint = amplitude * sign;

        if(new_setpoint != setpoint){

            if(refMode == SQUARE){
              
                setpoint = new_setpoint;
                ref = setpoint;     
            
            }else if(refMode == OPTIMAL){
              
                ts = now;
                z0 = ref;
                zf = new_setpoint;
                distance = zf - z0;

                // TODO
            }

        }

        if(ref != setpoint){

          t = now - ts;
          if(t <= T){

              uff = -u0;
              phiff = K_PHI * u0 * t;
              ref = z0 + K_PHI * K_V * u0 * t * t * t/6;
              
            }else if(t <= 3.0 * T){
      
              uff = u0;
              phiff = K_PHI * u0 * (t - 2 * T);
              ref = z0 - K_PHI * K_V * u0 * (t*t*t/6 - T*t*t + T*T*t - T*T*T/3);
              
            }else if (t <= 4.0 * T){

              uff = -u0;
              phiff = -K_PHI * u0 * (t - 4 * T);
              ref = z0 + K_PHI * K_V * u0 * (t*t*t/6 - 2*T*t*t + 8*T*T*t - 26*T*T*T/3);
              
            }else{
              
              uff = 0.0;
              phiff = 0.0;
              ref = setpoint;
              
            }
             
          }
        }

        timebase += h;
        ref_duration = timebase - millis();
        if(ref_duration > 0){

          vTaskDelay(1);
          
        }
    }
}


void setRef(double newRef){
  
  ref = newRef;
  
}

void setParChanged(){

  parChanged = true;
  
}

boolean getParChanged(){
  
  boolean val = parChanged;
  parChanged = false;
  return val;
  
  }
  
double referenceGetRef(){
  
  if (mode == MANUAL){

    return manual;
    
  }

  return ref;  
  }

double referenceGetPhiff(){
  
  if (mode == OPTIMAL){

    return phiff;
    
  }

  return 0.0;  
  }

double referenceGetUff(){
  
  if (mode == OPTIMAL){

    return uff;
    
  }

  return 0.0;
  
  }

 