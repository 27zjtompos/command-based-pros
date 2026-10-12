#pragma once

#include "command/subsystem.h"
#include "command/runCommand.h"

/**
 * Intake subsystem
 *
 * Allows controlling the percentage power of the intake subsystem
 */
class Intake : public Subsystem {
private:
	/**
	 * Store the necessary resources for this subsystem in the private section. We will control these resources in
	 * member methods
	 */
	pros::Motor* motor;
public:
	
	Intake(pros::Motor* motor)
		: motor(std::move(motor)) {
	}

	/**
	 * @brief Periodic runs every frame (10ms) by the command scheduler after the subsystem is registered.
	 *
	 * @note Periodic is very useful tasks such as PID that need to run every frame, but the target is only set once.
	 */
	void periodic() override {
		
	}

	// Create member functions to actuate the subsystem
	// EX: move the intake at a certain (signed) percentage of voltage.
	/**
	 * @brief This command moves the intake at a certain voltage potential requested as a percentage [-1, 1]
	 *
	 * @param pct Percent voltage for the intake in the range [-1, 1]
	 */
	void set_pct(const double pct) {

		// Scale [-1, 1] to [-12000 mV, 12000 mV]
		this->motor->move_voltage(pct * 12000.0);
	}

	void set_voltage(const double voltage) {
		this->motor->move_voltage(voltage * 1000.0);
	}

	RunCommand* pct(const double pct) {
		// Create a new RunCommand
		// The lambda body is called at every update, in this case setting the intake percentage
		return new RunCommand(
			[this, pct] () // Capture "this" and the percentage request
			{
				this->set_pct(pct); // Set the percentage of the intake to the request
			},
			{this} // Add "this", the pointer to this subsystem that is currently running.
			       // It is important to ensure that all subsystems that are being utilized in a command are properly
			       // Freed to allow that command to run.
			);
	}

	// Free any additional resources that are needed.
	~Intake(){
		delete motor;
	}
};