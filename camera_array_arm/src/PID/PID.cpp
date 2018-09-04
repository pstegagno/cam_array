
#include <PID/PID.hpp>


CameraArrayProject::PIDParameters::PIDParameters(){
    windup_guard = 0;
    proportional_gain = 0.0011;
    integral_gain = 0.0010;
    derivative_gain = 0.001;
    prev_error = 0;
    int_error = 0;
    control = 0;
}

CameraArrayProject::PIDParameters::PIDParameters(double wg, double pg, double ig, double dg, double pe, double ie, double c){
    windup_guard = wg;
    proportional_gain = pg;
    integral_gain = ig;
    derivative_gain = dg;
    prev_error = pe;
    int_error = ie;
    control = c;
}

 
CameraArrayProject::PID::PID(){
    windup_guard = 0;
    proportional_gain = 0.0011;
    integral_gain = 0.0010;
    derivative_gain = 0.001;
    prev_error = 0;
    int_error = 0;
    control = 0;
}


CameraArrayProject::PID::PID(PIDParameters &params){
    windup_guard = params.windup_guard;
    proportional_gain = params.proportional_gain;
    integral_gain = params.integral_gain;
    derivative_gain = params.derivative_gain;
    prev_error = params.prev_error;
    int_error = params.int_error;
    control = params.control;
}


CameraArrayProject::PID::PID(double wg, double pg, double ig, double dg, double pe, double ie, double c){
    windup_guard = wg;
    proportional_gain = pg;
    integral_gain = ig;
    derivative_gain = dg;
    prev_error = pe;
    int_error = ie;
    control = c;
}

      
 
void CameraArrayProject::PID::pid_update( double curr_error)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;
 
    // integration with windup guarding
    this->int_error += (curr_error);
    if (this->int_error < -(this->windup_guard))
        this->int_error = -(this->windup_guard);
    else if (this->int_error > this->windup_guard)
        this->int_error = this->windup_guard;
 
    // differentiation
    diff = ((curr_error - this->prev_error));
 
    // scaling
    p_term = (this->proportional_gain * curr_error);
    i_term = (this->integral_gain     * this->int_error);
    d_term = (this->derivative_gain   * diff);
 
    // summation of terms
    this->control = p_term + i_term + d_term;
 
    // save current error as previous error for next iteration
    this->prev_error = curr_error;
}




void CameraArrayProject::PID::pid_update( double curr_error, double curr_value)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;
 
    // integration with windup guarding
    this->int_error += (curr_error);
    if (this->int_error < -(this->windup_guard))
        this->int_error = -(this->windup_guard);
    else if (this->int_error > this->windup_guard)
        this->int_error = this->windup_guard;
 
    // differentiation
    diff = ((curr_error - this->prev_error));
 
    // scaling
    p_term = (this->proportional_gain *curr_value* curr_error);
    i_term = (this->integral_gain     *curr_value* this->int_error);
    d_term = (this->derivative_gain   *curr_value* diff);
 
    // summation of terms
    this->control = p_term + i_term + d_term;
 
    // save current error as previous error for next iteration
    this->prev_error = curr_error;
    
    
}
