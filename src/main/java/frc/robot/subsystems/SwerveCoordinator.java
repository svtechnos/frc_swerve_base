// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveCoordinator extends SubsystemBase {
  SwerveModule leftFrontModule;
  SwerveModule leftBackModule;
  SwerveModule rightFrontModule;
  SwerveModule rightBackModule;
  /** Creates a new SwerveCoordinator. */
  public SwerveCoordinator(SwerveModule leftFrontModule, SwerveModule leftBackModule, SwerveModule rightFrontModule, SwerveModule rightBackModule) {
    this.leftFrontModule = leftFrontModule;
    this.leftBackModule = leftBackModule;
    this.rightFrontModule = rightFrontModule;
    this.rightBackModule = rightBackModule;
  }
  public void inplaceTurn(double power){
    leftFrontModule.setDirection(135.0);
    leftBackModule.setDirection(45.0);
    rightFrontModule.setDirection(-45.0);
    rightBackModule.setDirection(-135.0);

    leftFrontModule.setSpeed(power);
    leftBackModule.setSpeed(power);
    rightFrontModule.setSpeed(power);
    rightBackModule.setSpeed(power);
  }
  public void translateTurn(double direction, double translatePower, double turnPower)
  {
      double turnAngle = turnPower * 45.0;
  
      // if the left front Module is in the front
      if (SwerveModule.closestAngle(direction, 135.0) >= 90.0)
      {
          leftFrontModule.setDirection(direction + turnAngle);
      }
      // if it's in the back
      else
      {
          leftFrontModule.setDirection(direction - turnAngle);
      }
      // if the left back Module is in the front
      if (SwerveModule.closestAngle(direction, 225.0) > 90.0)
      {
          leftBackModule.setDirection(direction + turnAngle);
      }
      // if it's in the back
      else
      {
          leftBackModule.setDirection(direction - turnAngle);
      }
      // if the right front Module is in the front
      if (SwerveModule.closestAngle(direction, 45.0) > 90.0)
      {
          rightFrontModule.setDirection(direction + turnAngle);
      }
      // if it's in the back
      else
      {
          rightFrontModule.setDirection(direction - turnAngle);
      }
      // if the right back Module is in the front
      if (SwerveModule.closestAngle(direction, 315.0) >= 90.0)
      {
          rightBackModule.setDirection(direction + turnAngle);
      }
      // if it's in the back
      else
      {
          rightBackModule.setDirection(direction - turnAngle);
      }
  
      leftFrontModule.setSpeed(translatePower);
      leftBackModule.setSpeed(translatePower);
      rightFrontModule.setSpeed(translatePower);
      rightBackModule.setSpeed(translatePower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
