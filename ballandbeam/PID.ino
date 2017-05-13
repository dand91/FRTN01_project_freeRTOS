
  double PID_I; // Integrator state
  double PID_v; // Desired control signal
  double PID_e; // Current control error
  double PID_eOld;
  double PID_D;
  double PID_ad;
  double PID_bd;
  double PID_y;
  double PID_yOld;

  double PIDParameters[8];

  void PID_init() {

    PIDParameters[0] = -0.055; // K
    PIDParameters[1] = 30.0; // Ti
    PIDParameters[2] = 0.5; // Td
    PIDParameters[3] = 4.0; // Tr
    PIDParameters[4] = 5.0; // N
    PIDParameters[5] = 1.0; // Beta
    PIDParameters[6] = 0.05; // H
    PIDParameters[7] = 0.0; // integratorOn

    PID_ad = PIDParameters[2] / (PIDParameters[2] + PIDParameters[4] * PIDParameters[6]);
    PID_bd = PIDParameters[0] * PID_ad * PIDParameters[4];
    PID_D = 0.0;
    PID_I = 0.0;
    PID_v = 0.0;
    PID_e = 0.0;
    PID_eOld = 0.0;
    PID_y = 0.0;
    PID_yOld = 0.0;
    
  }

  float PID_calculateOutput(double yNew, double yref) {

    PID_y = yNew;
    PID_e = yref - PID_y;
    PID_v = PIDParameters[0] * (PIDParameters[5] * yref - PID_y) + PID_I + PID_D; // I is 0.0 if integratorOn is false
    PID_eOld = PID_e;
    
    return PID_v;
    
  }

  void PID_updateState(float u) {
    
    if (PIDParameters[7] == 1.0) {
      PID_I = PID_I + (PIDParameters[0] * PIDParameters[6] / PIDParameters[1]) * PID_e + (PIDParameters[6] / PIDParameters[3]) * (u - PID_v);
    } else {
      PID_I = 0.0;
    }
    PID_yOld = PID_y;
    
  }

  long PID_getHMillis() {
    
    return (long)(PIDParameters[6] * 1000.0);
  
  }

  void PID_setParameters(double newParameters[]) {

    PIDParameters[0] = newParameters[0]; 
    PIDParameters[1] = newParameters[1]; 
    PIDParameters[2] = newParameters[2]; 
    PIDParameters[3] = newParameters[3]; 
    PIDParameters[4] = newParameters[4]; 
    PIDParameters[5] = newParameters[5]; 
    PIDParameters[6] = newParameters[6]; 
    PIDParameters[7] = newParameters[7]; 

    if ( PIParameters[5] == 0.0) {
      
      PID_I = 0.0;
      
    }
  }

  double* PID_getParameters(){

    return PIDParameters;
    
    }
    
  void PID_reset(){
    
      PID_I = 0;
      PID_D = 0;
 
    }

