
#ifndef PID_HPP
#define PID_HPP


class PID {
  public:
    double windup_guard;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double prev_error;
    double int_error;
    double control; 
 
    PID();
 
    void pid_update( double curr_error);

    void pid_update( double curr_error, double curr_value);

};


#endif

