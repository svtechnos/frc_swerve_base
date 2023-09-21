// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  WPI_TalonSRX turnEncoder;
  CANSparkMax turnMotor;
  CANSparkMax driveMotor;
  RelativeEncoder driveEncoder;
  double turnEncoderOffset;
  private double motorDirection = 1;
  /**
   * Creates a new swerve module
   * @param turnEncoder
   * @param turnMotor
   * @param driveMotor
   * @param driveEncoder
   * @param turnEncoderOffset
   */
  public SwerveModule(WPI_TalonSRX turnEncoder, CANSparkMax turnMotor, CANSparkMax driveMotor, RelativeEncoder driveEncoder, double turnEncoderOffset){
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
    this.turnEncoder = turnEncoder;
    this.driveEncoder = driveEncoder;
    this.turnEncoderOffset = turnEncoderOffset;
  }
  /**
   * Sets the speed of the motor
   * @param speed
   */
  public void setSpeed(double speed){
    driveMotor.set(speed*motorDirection);
  }
  /**
   * Sets the direction of the wheel
   * @param direction
   */
  public void setDirection(double direction){
    double currentAngle = currentAngle();
    double deltaAngle = -closestAngle(currentAngle, direction);
    double deltaAngleFlipped = -closestAngle(currentAngle, direction+180.0);
    if (Math.abs(deltaAngle) <= Math.abs(deltaAngleFlipped)){
      motorDirection = 1;
      turnMotor.set(clipSpeed((deadzone(deltaAngle, Constants.SwerveConstants.TURN_ANGLE_DEADZONE)*Constants.SwerveConstants.MODULE_ROTATION_P),Constants.SwerveConstants.CLIP_SPEED));
    }
    else {
      motorDirection = -1;
      turnMotor.set(clipSpeed((deadzone(deltaAngleFlipped, Constants.SwerveConstants.TURN_ANGLE_DEADZONE)*Constants.SwerveConstants.MODULE_ROTATION_P),Constants.SwerveConstants.CLIP_SPEED));
    }
  }
  /**
   * Gets smallest delta angle from the
   * current angle to the target angle
   * @param currentAngle
   * @param targetAngle
   * @return delta angle from -180 to 180 positive is counter-clockwise negative is clockwise
   */
  public static double closestAngle(double currentAngle, double targetAngle){
    double deltaAngle = (targetAngle - currentAngle)%360;
    deltaAngle = (Math.abs(deltaAngle) > 180.0)?-(Math.signum(deltaAngle) * 360.0) + deltaAngle:deltaAngle;
    return deltaAngle;
  }
  /**
   * sets a number to 0 if its in the deadzone
   * @param number
   * @param deadzone
   * @return number after deadzone check
   */
  public static double deadzone(double number, double deadzone){
    number = (Math.abs(number)<deadzone)?number=0.0:number;
    return number;
  }
  /**
   * Clips the speed based on the clipSpeed
   * @param speed
   * @param clipSpeed
   * @return The clipped speed
   */
  public static double clipSpeed(double speed, double clipSpeed){
    speed = (Math.abs(speed)>clipSpeed)?clipSpeed:speed;
    return speed;
  }
  /**
   * Gets current angle
   * @return Current angle in degrees
   */
  public double currentAngle(){
    return (((turnEncoder.getSelectedSensorPosition())*(360.0/4096.0))-turnEncoderOffset)%360;
  }
}