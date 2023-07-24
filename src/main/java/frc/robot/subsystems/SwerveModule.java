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
  PIDController pid;
  private double motorDirection = 1;

  public SwerveModule(WPI_TalonSRX turnEncoder, CANSparkMax turnMotor, CANSparkMax driveMotor, RelativeEncoder driveEncoder, double turnEncoderOffset, double kP, double kI, double kD){
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
    this.turnEncoder = turnEncoder;
    this.driveEncoder = driveEncoder;
    this.turnEncoderOffset = turnEncoderOffset;
    pid = new PIDController(kP, kI, kD);
    pid.reset()
  }
  public void setSpeed(double speed){
    driveMotor.set(speed*motorDirection);
  }
  
  public void setDirection(double direction){
    double currentAngle = currentAngle();
    double deltaAngle = -closestAngle(currentAngle, direction);
    double deltaAngleFlipped = -closestAngle(currentAngle, direction+180.0);

    turnMotor.set(pid.calculate(currentAngle, direction));

    if (Math.abs(deltaAngle) <= Math.abs(deltaAngleFlipped)){
      motorDirection = 1;
      //turnMotor.set(clipSpeed((deadzone(deltaAngle, Constants.SwerveConstants.TURN_ANGLE_DEADZONE)*Constants.SwerveConstants.MODULE_ROTATION_P),Constants.SwerveConstants.CLIP_SPEED));
    }
    else {
      motorDirection = -1;
      //turnMotor.set(clipSpeed((deadzone(deltaAngleFlipped, Constants.SwerveConstants.TURN_ANGLE_DEADZONE)*Constants.SwerveConstants.MODULE_ROTATION_P),Constants.SwerveConstants.CLIP_SPEED));
    }
  }
  public static double closestAngle(double currentAngle, double targetAngle){
    // get direction
    double deltaAngle = (targetAngle - currentAngle)%360;
    // convert from -360 to 360 to -180 to 180
    deltaAngle = (Math.abs(deltaAngle) > 180.0)?-(Math.signum(deltaAngle) * 360.0) + deltaAngle:deltaAngle;
    return deltaAngle;
  }
  public static double deadzone(double number, double deadzone){
    number = (Math.abs(number)<deadzone)?number=0.0:number;
    return number;
  }
  public static double clipSpeed(double speed, double clipSpeed){
    speed = (Math.abs(speed)>clipSpeed)?clipSpeed:speed;
    return speed;
  }
  public double currentAngle(){
    return (((turnEncoder.getSelectedSensorPosition())*(360.0/4096.0))-turnEncoderOffset)%360;
  }
}
