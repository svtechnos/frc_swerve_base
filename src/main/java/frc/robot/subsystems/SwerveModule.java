// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonSRX turnEncoder;
  private final CANSparkMax turnMotor;
  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final double turnEncoderOffset;
  private final PIDController turnPidController;
  /**
   * Creates a new swerve module
   * @param turnEncoder
   * @param turnMotor
   * @param driveMotor
   * @param driveEncoder
   * @param turnEncoderOffset
   */
  public SwerveModule(int turnEncoder_ID, int turnMotor_ID, int driveMotor_ID, double turnEncoderOffset){
    this.driveMotor = new CANSparkMax(driveMotor_ID, MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(turnMotor_ID, MotorType.kBrushless);
    this.turnEncoder = new WPI_TalonSRX(turnEncoder_ID);
    this.driveEncoder = driveMotor.getEncoder();
    this.turnPidController = new PIDController(Constants.SwerveConstants.MODULE_ROTATION_P, 0, 0);
    this.turnEncoderOffset = turnEncoderOffset;
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(40);
    turnMotor.setSmartCurrentLimit(30);
    turnPidController.enableContinuousInput(0, 360);
    turnPidController.setTolerance(1);
  }
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
  public double getCurrentAngle(){
    return (((turnEncoder.getSelectedSensorPosition())*(360.0/4096.0))-turnEncoderOffset)%360;
  }
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getCurrentAngle()));
  }
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(getCurrentAngle()));
  }
  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
  public void setSwerveModuleState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getSwerveModuleState().angle);
    driveMotor.set(state.speedMetersPerSecond);
    turnMotor.set(turnPidController.calculate(getCurrentAngle(), state.angle.getDegrees()));
  }
  public double[] getMotorsCurrent(){
    return new double[]{driveMotor.getOutputCurrent(),turnMotor.getOutputCurrent()};
  }
  public double[] getMotorsTemp(){
    return new double[]{driveMotor.getMotorTemperature(),turnMotor.getMotorTemperature()};
  }
}