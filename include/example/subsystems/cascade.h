#pragma once

#include "command/command.h"
#include "api.h"

/**
 * Cascade subsystem
 *
 * Allows controlling the percentage power of the intake subsystem
 */
class Cascade : public Subsystem {
public:

	pros::MotorGroup* motors;

	// HAVE A PID CLASS that is the default that is passed to when a call to maintain a lift height is called.
	
	Cascade(pros::MotorGroup* motors)
		: motors(std::move(motors)) {
		this->motors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
	}

	
	void periodic() override {
		// Nothing to put into periodic, as the PID will be implemented as a seperate command
	}

	

	void set_pct(double pct){
		this->motors->move_voltage(pct * 12000.0);
	}

	void set_voltage(double voltage){
		this->motors->move_voltage(voltage);
	}

	double get_pos(){
		return this->motors->get_position();
	};

	double get_vel(){
		return this->motors->get_actual_velocity();
	};


	RunCommand* pct(double pct) {
		return new RunCommand([this, pct]() { this->set_pct(pct); }, {this});
	}

	RunCommand* stop(){
		return this->pct(0);
	};

	RunCommand* volt(double voltage) {
		return new RunCommand([this, voltage]() { this->set_voltage(voltage); }, {this});
	}

	RunCommand* set_to_height(double height) {
		// Implement PID control loop that continously holds the lift up at a certain height.

		// Add a parameter that when the cascade reaches inside the set point, it finishes. This is useful for autos that need the lift
		// to raise for a certain height. Functional Command?
		return new RunCommand([this]() { this->set_voltage(0); }, {this});
	}



	~Cascade() override {
		delete motors;
	}
};