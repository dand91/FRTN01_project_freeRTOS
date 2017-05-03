
  double PI_I; // Integrator state
  double PI_v; // Desired control signal
  double PI_e; // Current control error

  double PIParameters[6];

  void PI_init() {

    PIParameters[0] = 2; // K
    PIParameters[1] = 30.0; // Ti
    PIParameters[2] = 1.0; // Tr
    PIParameters[3] = 0.1; // Beta
    PIParameters[4] = 0.1; // H
    PIParameters[5] = 1.0; // integratorOn
    
    PI_I = 0.0;
    PI_v = 0.0;
    PI_e = 0.0;
    
  }

  float PI_calculateOutput(double y, double yref) {

    PI_e = yref - y;
    PI_v = PIParameters[0] * (PIParameters[3] * yref - y) + PI_I; // I is 0.0 if integratorOn is false

    return PI_v;
  }

  void PI_updateState(float u) {
    
    if (PIParameters[5] == 1.0) {
      
      PI_I = PI_I + (PIParameters[0] * PIParameters[4] / PIParameters[1]) * PI_e + (PIParameters[4] / PIParameters[2]) * (u - PI_v);
      
    } else {
      
      PI_I = 0.0;
      
    }
  }

  long PI_getHMillis() {
    
    return (long)(PIParameters[4] * 1000.0);
    
  }

  double* PI_getParameters(){

    return PIParameters;
    
    }
    
  void PI_setParameters(double newParameters[]) {

    PIParameters[0] = newParameters[0]; // K
    PIParameters[1] = newParameters[1]; // Ti
    PIParameters[2] = newParameters[2]; // Tr
    PIParameters[3] = newParameters[3]; // Beta
    PIParameters[4] = newParameters[4]; // H
    PIParameters[5] = newParameters[5]; // integratorOn
    
    if ( PIParameters[5] == 0.0) {
      
      PI_I = 0.0;
      
    }
  }

  void PI_reset(){

      PI_I = 0.0;
      
    }

