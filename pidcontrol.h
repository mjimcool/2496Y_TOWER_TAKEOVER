#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PidControl{
    public:
    float currentPosition = 0;
    float error           = 0;
    float integral        = 0;
    float derivative      = 0;
    float last_error      = 0;
    int i_limit           = 0;
    int i_bound           = 100;
    
    float kP              = 0;
    float kI              = 0;
    float kD              = 0;

    float target          = 0;
    int   max_mV          = 12000;

    int   temp_output     = 0;
    int   output          = 0;
    int   slew_output     = 0;

    float slewError       = 0;
    float slewRate        = 0;

    Pidcontrol(float p, float i, float d, int iLimit){
        kP = p;
        kI = i;
        kD = d;
        i_limit = iLimit;
    }

    void pidInit(float p, float i, float d, int iLimit){
        kP = p;
        kI = i;
        kD = d;
        i_limit = iLimit;
    }

    void moveTo(){
        error = target - currentPosition;
        if(fabs(error) < i_bound && fabs(error) > 0){
            if(fabs(integral) < i_limit)
                integral += error;
            else{
                integral = sgn(integral)*i_limit;
            }   
        }
        else{
            integral = 0;
        }
        derivative = error - last_error;
        last_error = error;

        temp_output = kP*error + kI*integral + kD*derivative;

        output = fabs(temp_output) > max_mV ? sgn(temp_output)*max_mV : temp_output;
    }

    void slewOverride(){
        slewError = output - slew_output;

        if(fabs(slewError) >= slewRate)
        {
            slew_output += sgn(slewError)*slewRate;
        }
        else
        {
            slew_output += slewError;
        }
        if(fabs(output) > fabs(slew_output))
        {
            output = slew_output;
        }
    }

    void reset_variables(){
        currentPosition = 0;
        error           = 0;
        integral        = 0;
        derivative      = 0;
        last_error      = 0;
        target          = 0;
        temp_output     = 0;
        output          = 0;
        slew_output     = 0;
        slewError       = 0;
    }

    void reset_integral(){
        integral = 0;
    }
};

PidControl lift(28, 0.5, 0, 8000);
PidControl f_chassis(0, 0, 0, 0);//later initialized in drive()/pivot() functions
PidControl angle(5, 0, 0, 1000);
PidControl o_claw(600, 80, 0, 50);

#endif