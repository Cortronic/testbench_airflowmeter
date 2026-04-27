#ifndef PID_H
#define PID_H
#define LIBRARY_VERSION	1.2.2

class Pid {
  public:
    typedef enum {
      MANUAL = 0,
      AUTOMATIC = 1,
    } Mode;

    typedef enum {
      DIRECT = 0,
      REVERSE = 1,
    } Direction;

    typedef enum {
      P_ON_MEASURE = 0,
      P_ON_ERROR = 1,  
    } ProportionalOn;


    //commonly used functions ************************************************************************
    Pid(double *in, double *out, double *setpoint, // * constructor.  links the PID to the Input, Output, and 
      double kp, double ki, double kd,             //   Setpoint.  Initial tuning parameters are also set here.
      ProportionalOn, Direction);                  //   (overload for specifying proportional mode)
                                                  

    Pid(double *in, double *out, double *setpoint, // * constructor.  links the PID to the Input, Output, and 
      double kp, double ki, double kd,             //   Setpoint.  Initial tuning parameters are also set here
      Direction);     
	
    void setMode(Mode mode);                       // * sets PID to either Manual (0) or Auto (non-0)

    bool compute();                                // * performs the PID calculation.  it should be
                                                   //   called every time loop() cycles. ON/OFF and
                                                   //   calculation frequency can be set using SetMode
                                                   //   SetSampleTime respectively

    bool update();                                 // * performs the PID calculation.  it should be
                                                   //   called every sampletime ms.                                      

    void setOutputLimits(double min, double max); // * clamps the output to a specific range. 0-255 by default, but
										                              //   it's likely the user will want to change this depending on
										                              //   the application
	

    // available but not commonly used functions ********************************************************
    void setTunings(double kp, double ki,         // * While most users will set the tunings once in the 
      double kd);      	                          //   constructor, this function gives the user the option
                                                  //   of changing tunings during runtime for Adaptive control
    void setTunings(double kp, double ki,
      double kd, ProportionalOn);         	  

	  void  setControllerDirection(Direction);      // * Sets the Direction, or "Action" of the controller. DIRECT
										                              //   means the output will increase when error is positive. REVERSE
										                              //   means the opposite.  it's very unlikely that this will be needed
										                              //   once it is set in the constructor.

    void  setSampleTime(int sampleTime);          // * sets the frequency, in Milliseconds, with which 
                                                  // the PID calculation is performed.  default is 100
				
                                          
    // Display functions ****************************************************************
	  double          getKp();				// These functions query the pid for interal values.
	  double          getKi();				// They were created mainly for the pid front-end,
	  double          getKd();				// where it's important to know what is actually 
	  Mode            getMode();			// inside the PID.
	  Direction       getDirection();	//

  private:
	  void            initialize();
	
	  double          _dispKp;	      // * we'll hold on to the tuning parameters in user-entered 
	  double          _dispKi;	      //   format for display purposes
	  double          _dispKd;		    //
    
	  double          _kp;            // * (P)roportional Tuning Parameter
    double          _ki;            // * (I)ntegral Tuning Parameter
    double          _kd;            // * (D)erivative Tuning Parameter

	  Direction       _controllerDirection;
	  ProportionalOn  _pOn;

    double*         _input;         // * Pointers to the Input, Output, and Setpoint variables
    double*         _output;        //   This creates a hard link between the variables and the 
    double*         _setpoint;      //   PID, freeing the user from having to constantly tell us
                                    //   what these values are.  with pointers we'll just know.
			  
	  unsigned long   _sampleTime;
    unsigned long   _lastTime;
	  double          _outputSum, _lastInput;
	  
	  double          _outMin, _outMax;
	  bool            _inAuto, _pOnE;
};
#endif
