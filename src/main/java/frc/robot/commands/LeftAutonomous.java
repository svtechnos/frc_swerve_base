// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class LeftAutonomous extends SequentialCommandGroup {
  SwerveDrive swerveDrive;
  /**
   * Left routine(Charged up) for autonomous mode
   * @param swerveDrive
   */
  public LeftAutonomous(SwerveDrive swerveDrive) {
    addCommands(
      //place cone
      new DirectionDrive(swerveDrive,4,Constants.Directions.FORWARD,0.3)
    );
  }
}
