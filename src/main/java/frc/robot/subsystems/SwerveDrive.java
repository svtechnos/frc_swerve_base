// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveDrive extends SubsystemBase {
  // SwerveCoordinator
  public SwerveCoordinator SWERVE_COORDINATOR;
  // SwerveModules
  SwerveModule LEFT_FRONT_MODULE;
  SwerveModule LEFT_BACK_MODULE;
  SwerveModule RIGHT_FRONT_MODULE;
  SwerveModule RIGHT_BACK_MODULE;
  //Motors
  private static CANSparkMax LEFT_FRONT_DRIVE_MOTOR;
  private static CANSparkMax LEFT_BACK_DRIVE_MOTOR;
  private static CANSparkMax RIGHT_FRONT_DRIVE_MOTOR;
  private static CANSparkMax RIGHT_BACK_DRIVE_MOTOR;

  private static CANSparkMax LEFT_FRONT_TURN_MOTOR;
  private static CANSparkMax LEFT_BACK_TURN_MOTOR;
  private static CANSparkMax RIGHT_FRONT_TURN_MOTOR;
  private static CANSparkMax RIGHT_BACK_TURN_MOTOR;
  // Encoders
  public static RelativeEncoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static RelativeEncoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
  public static RelativeEncoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static RelativeEncoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;

  public static WPI_TalonSRX LEFT_FRONT_TURN_ENCODER;
  public static WPI_TalonSRX LEFT_BACK_TURN_ENCODER;
  public static WPI_TalonSRX RIGHT_FRONT_TURN_ENCODER;
  public static WPI_TalonSRX RIGHT_BACK_TURN_ENCODER;

  // Gyro
  public static Pigeon2 GYRO;
  
  public SwerveDrive() {
    //Motors
    LEFT_FRONT_DRIVE_MOTOR = new CANSparkMax(Constants.DeviceIDs.LEFT_FRONT_DRIVE_ID, MotorType.kBrushless);
    LEFT_BACK_DRIVE_MOTOR = new CANSparkMax(Constants.DeviceIDs.LEFT_BACK_DRIVE_ID, MotorType.kBrushless);
    RIGHT_FRONT_DRIVE_MOTOR = new CANSparkMax(Constants.DeviceIDs.RIGHT_FRONT_DRIVE_ID, MotorType.kBrushless);
    RIGHT_BACK_DRIVE_MOTOR = new CANSparkMax(Constants.DeviceIDs.RIGHT_BACK_DRIVE_ID, MotorType.kBrushless);
    LEFT_FRONT_DRIVE_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
    LEFT_BACK_DRIVE_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
    RIGHT_FRONT_DRIVE_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
    RIGHT_BACK_DRIVE_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.DRIVE_MOTOR_RAMP_RATE);
    
    LEFT_FRONT_TURN_MOTOR = new CANSparkMax(Constants.DeviceIDs.LEFT_FRONT_TURN_ID, MotorType.kBrushless);
    LEFT_BACK_TURN_MOTOR = new CANSparkMax(Constants.DeviceIDs.LEFT_BACK_TURN_ID, MotorType.kBrushless);
    RIGHT_FRONT_TURN_MOTOR = new CANSparkMax(Constants.DeviceIDs.RIGHT_FRONT_TURN_ID, MotorType.kBrushless);
    RIGHT_BACK_TURN_MOTOR = new CANSparkMax(Constants.DeviceIDs.RIGHT_BACK_TURN_ID, MotorType.kBrushless);
    LEFT_FRONT_TURN_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.TURN_MOTOR_RAMP_RATE);
    LEFT_BACK_TURN_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.TURN_MOTOR_RAMP_RATE);
    RIGHT_FRONT_TURN_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.TURN_MOTOR_RAMP_RATE);
    RIGHT_BACK_TURN_MOTOR.setOpenLoopRampRate(Constants.SwerveConstants.TURN_MOTOR_RAMP_RATE);
    // Encoders
    LEFT_FRONT_DRIVE_DISTANCE_ENCODER = LEFT_FRONT_TURN_MOTOR.getEncoder();
    LEFT_BACK_DRIVE_DISTANCE_ENCODER = LEFT_BACK_TURN_MOTOR.getEncoder();
    RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = RIGHT_FRONT_TURN_MOTOR.getEncoder();
    RIGHT_BACK_DRIVE_DISTANCE_ENCODER = RIGHT_BACK_TURN_MOTOR.getEncoder();

    LEFT_FRONT_TURN_ENCODER = new WPI_TalonSRX(Constants.DeviceIDs.LEFT_FRONT_TURN_ENCODER_ID);
    LEFT_BACK_TURN_ENCODER = new WPI_TalonSRX(Constants.DeviceIDs.LEFT_BACK_TURN_ENCODER_ID);
    RIGHT_FRONT_TURN_ENCODER = new WPI_TalonSRX(Constants.DeviceIDs.RIGHT_FRONT_TURN_ENCODER_ID);
    RIGHT_BACK_TURN_ENCODER = new WPI_TalonSRX(Constants.DeviceIDs.RIGHT_BACK_TURN_ENCODER_ID);
    // Gyro
    GYRO = new Pigeon2(Constants.DeviceIDs.GYRO_DEVICE_ID, "rio");

    // SwerveDriveModules
    LEFT_FRONT_MODULE = new SwerveModule(LEFT_FRONT_TURN_ENCODER, LEFT_FRONT_TURN_MOTOR, LEFT_FRONT_DRIVE_MOTOR, LEFT_FRONT_DRIVE_DISTANCE_ENCODER, Constants.Offsets.LEFT_FRONT_TURN_ENCODER_OFFSET);
    LEFT_BACK_MODULE = new SwerveModule(LEFT_BACK_TURN_ENCODER, LEFT_BACK_TURN_MOTOR, LEFT_BACK_DRIVE_MOTOR, LEFT_BACK_DRIVE_DISTANCE_ENCODER, Constants.Offsets.LEFT_BACK_TURN_ENCODER_OFFSET);
    RIGHT_FRONT_MODULE = new SwerveModule(RIGHT_FRONT_TURN_ENCODER, RIGHT_FRONT_TURN_MOTOR, RIGHT_FRONT_DRIVE_MOTOR, RIGHT_FRONT_DRIVE_DISTANCE_ENCODER, Constants.Offsets.RIGHT_FRONT_TURN_ENCODER_OFFSET);
    RIGHT_BACK_MODULE = new SwerveModule(RIGHT_BACK_TURN_ENCODER, RIGHT_BACK_TURN_MOTOR, RIGHT_BACK_DRIVE_MOTOR, RIGHT_BACK_DRIVE_DISTANCE_ENCODER, Constants.Offsets.RIGHT_BACK_TURN_ENCODER_OFFSET);
    // SwerveCoordinator
    SWERVE_COORDINATOR = new SwerveCoordinator(LEFT_FRONT_MODULE, LEFT_BACK_MODULE, RIGHT_FRONT_MODULE, RIGHT_BACK_MODULE);
  }
  public void resetDriveEncoders(){
    LEFT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
    LEFT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
    RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.setPosition(0);
    RIGHT_BACK_DRIVE_DISTANCE_ENCODER.setPosition(0);
  }

  public boolean reachedDistance(double distanceMeters){
    return (LEFT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition()>distanceMeters)&&(LEFT_BACK_DRIVE_DISTANCE_ENCODER.getPosition()>distanceMeters)&&(RIGHT_FRONT_DRIVE_DISTANCE_ENCODER.getPosition()>distanceMeters)&&(RIGHT_BACK_DRIVE_DISTANCE_ENCODER.getPosition()>distanceMeters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
