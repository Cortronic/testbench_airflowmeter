#ifndef pid_h
#define pid_h
#define LIBRARY_VERSION	1.2.2

class Pid {
  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    Pid(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    Pid(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void setMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void setOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void setTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void setTunings(double, double,       // * overload for specifying proportional mode
                    double, int);         	  

	void setControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void setSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double getKp();						  // These functions query the pid for interal values.
	double getKi();						  //  they were created mainly for the pid front-end,
	double getKd();						  // where it's important to know what is actually 
	int getMode();						  //  inside the PID.
	int getDirection();					  //

  private:
	void initialize();
	
	double _dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double _dispKi;				//   format for display purposes
	double _dispKd;				//
    
	double _kp;                  // * (P)roportional Tuning Parameter
    double _ki;                  // * (I)ntegral Tuning Parameter
    double _kd;                  // * (D)erivative Tuning Parameter

	int _controllerDirection;
	int _pOn;

    double *_input;              // * Pointers to the Input, Output, and Setpoint variables
    double *_output;             //   This creates a hard link between the variables and the 
    double *_setpoint;           //   PID, freeing the user from having to constantly tell us
                                 //   what these values are.  with pointers we'll just know.
			  
	unsigned long _lastTime;
	double _outputSum, _lastInput;

	unsigned long _sampleTime;
	double _outMin, _outMax;
	bool _inAuto, _pOnE;
};
#endif
