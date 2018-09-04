
#ifndef PID_HPP
#define PID_HPP


namespace CameraArrayProject{
  
struct PIDParameters{
public:
    double windup_guard;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double prev_error;
    double int_error;
    double control;
    
    PIDParameters();
    
    PIDParameters(double wg, double pg, double ig, double dg, double pe, double ie, double c);

};

class PID: public PIDParameters{
  public:
 
    PID();
    
    PID(PIDParameters &params);
    
    PID(double wg, double pg, double ig, double dg, double pe, double ie, double c);
 
    void pid_update( double curr_error);

    void pid_update( double curr_error, double curr_value);

};

};// end namespace CameraArrayProject

#endif

