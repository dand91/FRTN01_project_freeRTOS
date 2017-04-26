// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;
	private double I;
	private double D;
	private double ad;
	private double bd;
	private double yCurr;
	private double yOld;
	private double v; // desired control signal
	private double e; // current control error
	private double eOld;
	
	// Constructor
	public PID(String name){
		PIDParameters p = new PIDParameters();
		p.K = -0.1;
		p.Td = 1.5;
		p.H = 0.02;
		p.N = 5.0;
		p.Beta = 1.0;
		p.Ti = 0.0;
		p.Tr = 10.0;
		p.integratorOn = false;
		//new PIDGUI(this,p,name);
		setParameters(p);
		this.ad = p.Td / (p.Td + p.N * p.H);
		this.bd = p.K * ad * p.N;
		this.D = 0.0;
		this.I = 0.0;
		this.v = 0.0;
		this.e = 0.0;
		this.yOld = 0.0;
		this.yCurr = 0.0;
	}
	
	// Calculates the control signal v.
	// Called from BallAndBeamRegul.
	public synchronized double calculateOutput(double y, double yref){
		this.yCurr = y;
		this.e = yref - y;
		this.D = ad * this.D + bd * (e - eOld);
		this.v = p.K * (p.Beta * yref - y) + I + D;
		this.eOld = e;
		return v;
	}
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BallAndBeamRegul.
	public synchronized void updateState(double u){
		if(p.integratorOn){
			// Forward Euler approximation with tracking
		    I = I + (p.K * p.H / p.Ti) * e + (p.H / p.Tr) * (u-v);
		    } else{
				I = 0.0;
			}
			yOld = yCurr;
	}
	
	// Returns the sampling interval expressed as a long.
	// Explicit type casting needed.
	public synchronized long getHMillis(){
		return (long) (p.H * 1000.0); // because H is in [s]
	}
	
	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters){
		p = (PIDParameters) newParameters.clone();
		this.ad = p.Td / (p.Td + p.N * p.H);
		this.bd = p.K * ad * p.N;
		if(!p.integratorOn){
			I = 0.0;
		}
	}
	
	  /** Returns the parameters of the controller. */
    public synchronized PIDParameters getParameters(){
		return(PIDParameters) p.clone();
	}

    /** Reset the states of the controller, called when changing mode in Regul. */
    public synchronized void reset(){
		this.I = 0.0;
		this.D = 0.0;
	}
}
