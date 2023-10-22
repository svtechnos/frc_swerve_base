// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Kinematics;


public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  private final Pigeon2 gyro;
  private final SwerveDriveOdometry odometry;
  /**
   * Main swerve drive subsystem
   */
  public SwerveDrive() {
    // Gyro
    gyro = new Pigeon2(Constants.DeviceIDs.GYRO, "rio");
    // SwerveDriveModules
    frontLeftModule = new SwerveModule(Constants.DeviceIDs.LEFT_FRONT_TURN_ENCODER, Constants.DeviceIDs.LEFT_FRONT_TURN, Constants.DeviceIDs.LEFT_FRONT_DRIVE, Constants.Offsets.LEFT_FRONT_TURN_ENCODER_OFFSET);
    frontRightModule = new SwerveModule(Constants.DeviceIDs.RIGHT_FRONT_TURN_ENCODER, Constants.DeviceIDs.RIGHT_FRONT_TURN, Constants.DeviceIDs.RIGHT_FRONT_DRIVE, Constants.Offsets.RIGHT_FRONT_TURN_ENCODER_OFFSET);
    backLeftModule = new SwerveModule(Constants.DeviceIDs.LEFT_BACK_TURN_ENCODER, Constants.DeviceIDs.LEFT_BACK_TURN, Constants.DeviceIDs.LEFT_BACK_DRIVE, Constants.Offsets.LEFT_BACK_TURN_ENCODER_OFFSET);
    backRightModule = new SwerveModule(Constants.DeviceIDs.RIGHT_BACK_TURN_ENCODER, Constants.DeviceIDs.RIGHT_BACK_TURN, Constants.DeviceIDs.RIGHT_BACK_DRIVE, Constants.Offsets.RIGHT_BACK_TURN_ENCODER_OFFSET);
    // Odometry
    odometry = new SwerveDriveOdometry(Constants.Kinematics.kSwerveDriveKinematics, new Rotation2d(gyro.getYaw()), getModulePositions());
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      frontLeftModule.getPosition(), 
      frontRightModule.getPosition(), 
      backLeftModule.getPosition(),
      backRightModule.getPosition()};
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public void stopModules() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      frontLeftModule.getSwerveModuleState(),
      frontRightModule.getSwerveModuleState(),
      backLeftModule.getSwerveModuleState(),
      backRightModule.getSwerveModuleState()};
  }
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    setModuleStates(Constants.Kinematics.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Kinematics.kPhysicalMaxSpeedMetersPerSecond);
    frontLeftModule.setSwerveModuleState(desiredStates[0]);
    frontRightModule.setSwerveModuleState(desiredStates[1]);
    backLeftModule.setSwerveModuleState(desiredStates[2]);
    backRightModule.setSwerveModuleState(desiredStates[3]);
  }
  public Rotation2d getOdometryAngle() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }
  public void resetYaw() {
    gyro.setYaw(0);
  }
  public double getYaw() {
    return gyro.getYaw();
  }
  public double getPitch() {
    return gyro.getPitch();
  }
  public double getRoll() {
    return gyro.getRoll();
  }
  /**
   * Resets all drive encoders
   */
  public void resetDriveEncoders(){
    frontLeftModule.resetDriveEncoder();
    frontRightModule.resetDriveEncoder();
    backLeftModule.resetDriveEncoder();
    backRightModule.resetDriveEncoder();
  }
  /**
   * Checks if the robot has reached a certain distance in meters
   * @param distanceMeters
   * @return If the robot has reached the given distance
   */
  public boolean reachedDistance(double distanceMeters){
    distanceMeters=Math.abs(distanceMeters);
    double frontLeftCurrentPosition=Math.abs(frontLeftModule.getDrivePosition());
    double frontRightCurrentPosition=Math.abs(frontRightModule.getDrivePosition());
    double backLeftCurrentPosition=Math.abs(backLeftModule.getDrivePosition());
    double backRightCurrentPosition=Math.abs(backRightModule.getDrivePosition());
    return (frontLeftCurrentPosition>distanceMeters)
         &&(frontRightCurrentPosition>distanceMeters)
         &&(backLeftCurrentPosition>distanceMeters)
         &&(backRightCurrentPosition>distanceMeters);
  }

  @Override
  public void periodic() {}
}
