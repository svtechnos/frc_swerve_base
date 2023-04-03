// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class ChargeStation extends CommandBase {
  SwerveDrive swervedrive;
  double levelRoll;
  boolean starting;
  double startingYaw;

  public ChargeStation(SwerveDrive swervedrive) {
    this.swervedrive = swervedrive;
    addRequirements(swervedrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    levelRoll = SwerveDrive.GYRO.getRoll();
    startingYaw = SwerveDrive.GYRO.getYaw();
    starting=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tiltError = SwerveDrive.GYRO.getRoll() - levelRoll;
    double dirError = SwerveDrive.GYRO.getYaw() - startingYaw;
    if(tiltError>Constants.ChargeStationConstants.START_CLIMB_ANGLE) {starting=false;}
    if(starting){swervedrive.SWERVE_COORDINATOR.translateTurn(Constants.Directions.FORWARD, Constants.ChargeStationConstants.START_SPEED, dirError*Constants.ChargeStationConstants.YAW_P);}
    else{
      if(tiltError>Constants.ChargeStationConstants.DEADZONE){swervedrive.SWERVE_COORDINATOR.translateTurn(Constants.Directions.FORWARD, tiltError*Constants.ChargeStationConstants.TILT_P, dirError*Constants.ChargeStationConstants.YAW_P);}
      else if(tiltError<-Constants.ChargeStationConstants.DEADZONE){swervedrive.SWERVE_COORDINATOR.translateTurn(Constants.Directions.BACKWARD, tiltError*Constants.ChargeStationConstants.TILT_P, dirError*Constants.ChargeStationConstants.YAW_P);}
      else {swervedrive.SWERVE_COORDINATOR.lockPosition();}
    }
  }
  @Override
  public void end(boolean interrupted) {swervedrive.SWERVE_COORDINATOR.translateTurn(0, 0, 0);}
  @Override
  public boolean isFinished() {return false;}
}