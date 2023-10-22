// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class TeleopMovement extends CommandBase {
  XboxController joystick;
  SwerveDrive swerveDrive;
  double direction=0;
  /**
   * Teleop mode command, lets the user have full control over the robots movement
   * @param swerveDrive
   * @param joystick
   */
  public TeleopMovement(SwerveDrive swerveDrive, XboxController joystick) {
    this.joystick = joystick;
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }
  @Override
  public void initialize(){swerveDrive.resetYaw();}
  @Override
  public void execute(){
    swerveDrive.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(joystick.getLeftX(),joystick.getLeftY(),joystick.getRightX()), new Rotation2d(swerveDrive.getYaw())));
  }
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setModuleStates(new ChassisSpeeds(0,0,0));
  }
  
  @Override
  public boolean isFinished() {return false;}
}
