#ifdef ODE_EXT

#include "BaseLib/OdeExt/PDController.h"



PDController::PDController()
{
	input_ = 0;
	last_input_ = 0;
	d_input_ = 0;
	target_ = 0;
	//kp_ = 10000.0;
	kp_ = 500.0;
	kd_ = 30.00;
	error_ = 0;
	last_error_ = 0;
	d_error_ = 0;

	flag_on_off_ = false;
	flag_first_step_after_on_ = true;
}

void
PDController::On()
{
	flag_on_off_ = true;
	flag_first_step_after_on_ = true;
}

void
PDController::Off()
{
	flag_on_off_ = false;
}

double
PDController::Step(double ellipsed_time)
{
	if ( !flag_on_off_ ) return 0;

	last_error_ = error_;
	error_ = CalculError();

	if ( flag_first_step_after_on_ ) 
	{	
		last_error_ = error_;
		last_input_ = input_;
		flag_first_step_after_on_ = false;
	}
	d_error_ = (error_ - last_error_)/ellipsed_time;
	d_input_ = (input_ - last_input_)/ellipsed_time;

	output_ = kp_ * error_ 
			+ kd_ * d_error_;

	return output_;
}

double
PDController::CalculError()
{ 
	return target_ - input_; 
}


double
PDController::output2() const
{	return kp_ * error_ - kd_ * d_input_;
}








#endif
