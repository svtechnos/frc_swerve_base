// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DirectionDrive extends CommandBase {
  /** Creates a new DirectionDrive. */
  SwerveDrive swerveDrive;
  double distanceMeters;
  double direction;
  double speed;
  public DirectionDrive(SwerveDrive swerveDrive, double distanceMeters, double direction, double speed) {
    this.swerveDrive = swerveDrive;
    this.direction = direction;
    this.distanceMeters = distanceMeters;
    this.speed = speed;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {swerveDrive.reset();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {swerveDrive.SWERVE_COORDINATOR.translateTurn(direction,speed,0);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {swerveDrive.SWERVE_COORDINATOR.translateTurn(direction,0,0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {return swerveDrive.reached(distanceMeters);}
}