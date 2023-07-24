// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class TeleopMovement extends CommandBase {
  Joystick joystick;
  SwerveDrive swerveDrive;
  double direction=0;
  /** Creates a new TeleopMovement. */
  public TeleopMovement(SwerveDrive swerveDrive, Joystick joystick) {
    this.joystick = joystick;
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){SwerveDrive.GYRO.setYaw(0);}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(joystick.getMagnitude()>Constants.SwerveConstants.MOVEMENT_SPEED_DEADZONE){
      direction = (joystick.getDirectionDegrees()<0)?-joystick.getDirectionDegrees():-joystick.getDirectionDegrees()+360;
    }
    
    double modifier=((-joystick.getThrottle()+1+Constants.SwerveConstants.GAIN_BIAS)/(2+Constants.SwerveConstants.GAIN_BIAS));
    //modifier=0;
    //swerveDrive.SWERVE_COORDINATOR.swerveMove(direction-SwerveDrive.GYRO.getYaw(), joystick.getMagnitude()*modifier, joystick.getZ(),modifier);
    swerveDrive.SWERVE_COORDINATOR.swerveMove(joystick.getX(), joystick.getY(), joystick.getZ(),modifier);

    //swerveDrive.SWERVE_COORDINATOR.swerveMove(direction-SwerveDrive.GYRO.getYaw(), joystick.getMagnitude()*poten, joystick.getZ()/2);
    //swerveDrive.SWERVE_COORDINATOR.swerveMove(direction, joystick.getMagnitude(), joystick.getZ());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {swerveDrive.SWERVE_COORDINATOR.swerveMove(direction,0,0,0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
