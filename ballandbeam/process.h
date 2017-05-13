/*
 *      double h2 = Math.pow(h, 2.0);
      double h3 = Math.pow(h, 3.0);
      newState[0] = state[0] + h*state[1] - kX*h2*state[2]/2.0 - 
    kX*kPhi*h3*input[0]/6.0;
      newState[1] = state[1] - kX*h*state[2] - kX*kPhi*h2*input[0]/2.0;
      newState[2] = state[2] + kPhi*h*input[0];
 */

double kPhi=4.4; //processkonstant för vinkel
double kX = 7.0; //processkonstant för kulan

double angle;
double pos;
double ctl;

double x1, x2, x3; // states

void simulationStep(double h) {

  double x1_t = 0;
  double x2_t = 0;
  double x3_t = 0;
  
  double h2 = h*h;
  double h3 = h*h*h;
  x1_t = x1 + h*x2 - kX*h2*x3/2.0 - kX*kPhi*h3*ctl/6.0;
  x2_t = x2 - kX*h*x3 - kX*kPhi*h2*ctl/2.0;
  x3_t = x3 + kPhi*h*ctl;
  
  x1 = x1_t;
  x2 = x2_t;
  x3 = x3_t;
  
  angle = x3;
  pos = x2;

}

double readAngle(){
  return angle;
}

double readPosition(){
  return pos;
}

void writeControlSignal(double u){
  ctl = u;
}

