// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import com.ctre.phoenix.sensors.Pigeon2;

public class TeleopMovement extends CommandBase {
  Joystick joystick;
  SwerveDrive swerveDrive;
  Pigeon2 Gyro;
  /** Creates a new TeleopMovement. */
  public TeleopMovement(SwerveDrive swerveDrive, Joystick joystick,Pigeon2 GYRO) {
    this.joystick = joystick;
    this.swerveDrive = swerveDrive;
    this.Gyro=GYRO;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    swerveDrive.SWERVE_COORDINATOR.translateTurn(joystick.getDirectionDegrees()-Gyro.getDirectionDegrees(), joystick.getMagnitude(), joystick.getZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {swerveDrive.SWERVE_COORDINATOR.translateTurn(0,0,0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
