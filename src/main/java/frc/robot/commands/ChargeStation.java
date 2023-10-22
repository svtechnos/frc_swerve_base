// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class ChargeStation extends CommandBase {
  SwerveDrive swerveDrive;
  double levelRoll;
  boolean starting;
  double startingYaw;
  double direction;
  /**
   * Command the balances the robot on the charging station
   * @param swervedrive
   * @param direction Starting direction
   */
  public ChargeStation(SwerveDrive swerveDrive, double direction) {
    this.direction=direction;
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    levelRoll = swerveDrive.getRoll();
    startingYaw = swerveDrive.getYaw();
    starting=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tiltError = swerveDrive.getRoll() - levelRoll;
    double dirError = swerveDrive.getYaw() - startingYaw;
    if(Math.abs(tiltError)>Constants.ChargeStationConstants.START_CLIMB_ANGLE) {starting=false;}
    if(starting){
      swerveDrive.setModuleStates(new ChassisSpeeds(0,1, dirError*Constants.ChargeStationConstants.YAW_P));
    }
    else{
      if(tiltError>Constants.ChargeStationConstants.DEADZONE){
        swerveDrive.setModuleStates(new ChassisSpeeds(0,-Math.abs(tiltError)*Constants.ChargeStationConstants.TILT_P, dirError*Constants.ChargeStationConstants.YAW_P));
      }
      else if(tiltError<-Constants.ChargeStationConstants.DEADZONE){
        swerveDrive.setModuleStates(new ChassisSpeeds(0,Math.abs(tiltError)*Constants.ChargeStationConstants.TILT_P, dirError*Constants.ChargeStationConstants.YAW_P));
      }
    }
  }
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setModuleStates(new ChassisSpeeds(0,0,0));
  }
  @Override
  public boolean isFinished() {return false;}
}