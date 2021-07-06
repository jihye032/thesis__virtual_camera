#ifdef ODE_EXT
#pragma once

class PDController
{
public:
	PDController();

	void On();
	void Off();
	double Step(double ellipsed_time);

	void target(double t) { target_ = t; }
	void input(double c) { input_ = c; }

	double target() const { return target_; }
	double input() const { return input_; }
	double error() const { return error_; }
	double d_error() const { return d_error_; }
	double output() const { return output_; }

	void kp(double v) { kp_ = v; }
	void kd(double v) { kd_ = v; }

	double kp() const { return kp_; }
	double kd() const { return kd_; }

	double output2() const;

protected:
	inline double CalculError();
	
protected:
	double target_;
	double input_;
	double last_input_;
	double d_input_;
	double error_;
	double last_error_;
	double d_error_;
	double output_;

	double kp_;
	double kd_;

	bool flag_on_off_;
	bool flag_first_step_after_on_;
};


#endif
