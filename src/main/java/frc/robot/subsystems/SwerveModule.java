// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  WPI_TalonSRX turnEncoder;
  CANSparkMax turnMotor;
  CANSparkMax driveMotor;
  private double motorDirection = 1;

  public SwerveModule(WPI_TalonSRX turnEncoder, CANSparkMax turnMotor, CANSparkMax driveMotor){
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
    this.turnEncoder = turnEncoder;
  }
  public void setSpeed(double speed){
    driveMotor.set(speed*motorDirection);
  }
  public void setDirection(double direction){
    double currentAngle = turnEncoder.getSelectedSensorPosition()*0.087890625;
    double deltaAngle = closestAngle(currentAngle, direction);
    double deltaAngleFlipped = closestAngle(currentAngle, direction+180);
    if (Math.abs(deltaAngle) <= Math.abs(deltaAngleFlipped)){motorDirection = 1;turnMotor.set(deltaAngle*Constants.SwerveConstants.MODULE_ROTATION_P);}
    else {motorDirection = -1;turnMotor.set(deltaAngleFlipped*Constants.SwerveConstants.MODULE_ROTATION_P);}
  }
  public static double closestAngle(double currentAngle, double targetAngle){
    // get direction
    double dir = (targetAngle%360.0) - (currentAngle%360.0);
    // convert from -360 to 360 to -180 to 180
    dir = (Math.abs(dir) > 180.0)?-(Math.signum(dir) * 360.0) + dir:dir;
    return dir;
  }
}
