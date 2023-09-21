// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DirectionDrive extends CommandBase {
  SwerveDrive swerveDrive;
  double distanceMeters;
  double direction;
  double speed;
  double startingYaw;
  /**
   * Drives in the direction that you give, with the speed that you give, for the distance you give it to drive
   * @param swerveDrive
   * @param distanceMeters
   * @param direction
   * @param speed
   */
  public DirectionDrive(SwerveDrive swerveDrive, double distanceMeters, double direction, double speed) {
    this.swerveDrive = swerveDrive;
    this.direction = direction;
    this.distanceMeters = distanceMeters;
    this.speed = speed;
    addRequirements(swerveDrive);
  }
  @Override
  public void initialize() {    
    startingYaw = SwerveDrive.GYRO.getYaw();
    swerveDrive.resetDriveEncoders();
  }
  @Override
  public void execute() {
    double dirError = SwerveDrive.GYRO.getYaw() - startingYaw;
    swerveDrive.SWERVE_COORDINATOR.translateTurn(direction,speed, dirError*Constants.DirectionDriveConstants.YAW_P);
  }
  @Override
  public void end(boolean interrupted) {swerveDrive.SWERVE_COORDINATOR.translateTurn(direction,0,0);}
  @Override
  public boolean isFinished() {return swerveDrive.reachedDistance(distanceMeters);}
}