// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Monitor extends SubsystemBase {
  // Drive motor temperatures
  double LEFT_FRONT_DRIVE_TEMPREATURE;
  double RIGHT_FRONT_DRIVE_TEMPREATURE;
  double LEFT_BACK_DRIVE_TEMPREATURE;
  double RIGHT_BACK_DRIVE_TEMPREATURE;
  // Turn motor temperatures
  double LEFT_FRONT_TURN_TEMPREATURE;
  double RIGHT_FRONT_TURN_TEMPREATURE;
  double LEFT_BACK_TURN_TEMPREATURE;
  double RIGHT_BACK_TURN_TEMPREATURE;

  /** Creates a new Monitor. */
  public Monitor() {
    SwerveDrive.LEFT_FRONT_DRIVE_MOTOR.getMotorTemperature();
    SwerveDrive.RIGHT_FRONT_DRIVE_MOTOR.getMotorTemperature();
    SwerveDrive.LEFT_BACK_DRIVE_MOTOR.getMotorTemperature();
    SwerveDrive.RIGHT_BACK_DRIVE_MOTOR.getMotorTemperature();
  }

  @Override
  public void periodic() {
    
  }
}
