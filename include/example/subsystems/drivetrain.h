#pragma once

#include "command/command.h"
#include "command/commandController.h"
#include "api.h"

/**
 * Drivetrain subsystem
 *
 * Allows controlling the percentage power of the intake subsystem
 */
class Drivetrain : public Subsystem {
public:
	/**
	 * Store the necessary resources for this subsystem in the private section. We will control these resources in
	 * member methods
	 */
	pros::MotorGroup* left_drive;
	pros::MotorGroup* right_drive;

	pros::Imu* imu;

	double x;
	double y;
	double theta;
	
	Drivetrain(pros::MotorGroup* left_drive, pros::MotorGroup* right_drive, pros::Imu* imu)
		: left_drive(std::move(left_drive)), right_drive(std::move(right_drive)), imu(std::move(imu)) {
		this->left_drive->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    	this->right_drive->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

		this->x = 0;
		this->y = 0;
		this->theta = 0;
	}

	
	void periodic() override {
		// Need to implement Odometry.
	}

	

	void set_pct(double leftPct, double rightPct){

		this->left_drive->move_voltage(leftPct * 12000.0);
		this->right_drive->move_voltage(rightPct * 12000.0);
	}

	void set_voltage(double left_voltage, double right_voltage){
		this->left_drive->move_voltage(left_voltage * 1000.0);
		this->right_drive->move_voltage(right_voltage * 1000.0);
	}

	double get_left_pos(){
		return this->left_drive->get_position();
	};
	double get_right_pos(){
		return this->right_drive->get_position();
	};

	double get_left_vel(){
		return this->left_drive->get_actual_velocity();
	};
	double get_right_vel(){
		return this->right_drive->get_actual_velocity();
	};

	RunCommand* stop(){
		return this->pct(0, 0);
	};

	RunCommand* tank(CommandController &primary) {
		return new RunCommand(
			[this, primary]() mutable {
				this->set_pct(primary.get_analog(ANALOG_LEFT_Y) / 127.0,
								primary.get_analog(ANALOG_RIGHT_Y) / 127.0);
			},
			{this});
	}

	RunCommand* arcade(CommandController &primary) {
		return new RunCommand(
			[this, primary]() mutable {
				this->set_pct((primary.get_analog(ANALOG_LEFT_Y) + primary.get_analog(ANALOG_RIGHT_X)) / 127.0,
								(primary.get_analog(ANALOG_LEFT_Y) - primary.get_analog(ANALOG_RIGHT_X)) /
								127.0);
			},
			{this});
	}

	RunCommand* pct(double left, double right) {
		return new RunCommand([this, left, right]() { this->set_pct(left, right); }, {this});
	}

	RunCommand* volt(double left, double right) {
		return new RunCommand([this, left, right]() { this->set_voltage(left, right); }, {this});
	}


	~Drivetrain() override {
		delete this->left_drive;
		delete this->right_drive;
		delete this->imu;
	}
};