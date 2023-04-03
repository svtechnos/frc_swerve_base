// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveCoordinator extends SubsystemBase {
  public SwerveModule leftFrontModule;
  public SwerveModule leftBackModule;
  public SwerveModule rightFrontModule;
  public SwerveModule rightBackModule;
  /** Creates a new SwerveCoordinator. */
  public SwerveCoordinator(SwerveModule leftFrontModule, SwerveModule leftBackModule, SwerveModule rightFrontModule, SwerveModule rightBackModule) {
    this.leftFrontModule = leftFrontModule;
    this.leftBackModule = leftBackModule;
    this.rightFrontModule = rightFrontModule;
    this.rightBackModule = rightBackModule;
  }
  public void lockPosition() {
    leftFrontModule.setDirection(135.0);
    leftBackModule.setDirection(45.0);
    rightFrontModule.setDirection(-45.0);
    rightBackModule.setDirection(-135.0);
  }
  public void inplaceTurn(double power){
    leftFrontModule.setDirection(135.0);
    leftBackModule.setDirection(45.0);
    rightFrontModule.setDirection(-45.0);
    rightBackModule.setDirection(-135.0);
    //might have to fix stuff
    leftFrontModule.setSpeed(power);
    leftBackModule.setSpeed(power);
    rightFrontModule.setSpeed(power);
    rightBackModule.setSpeed(power);
  }
  public void translateTurn(double direction, double translatePower, double twistPower){
      // turnAngle goes from 45 to -45
      double twistAngle = twistPower * 45.0;

      // If the left front Module is in the front
      if ((Math.abs(SwerveModule.closestAngle(direction, 135.0))) >= 90.0) {leftFrontModule.setDirection(direction + twistAngle);}
      // if it's in the back
      else {leftFrontModule.setDirection(direction - twistAngle);}

      // If the left back Module is in the front
      if ((Math.abs(SwerveModule.closestAngle(direction, 225.0))) > 90.0) {leftBackModule.setDirection(direction + twistAngle);}
      // if it's in the back
      else {leftBackModule.setDirection(direction - twistAngle);}

      // If the right front Module is in the front
      if ((Math.abs(SwerveModule.closestAngle(direction, 45.0))) > 90.0) {rightFrontModule.setDirection(direction + twistAngle);}
      // if it's in the back
      else {rightFrontModule.setDirection(direction - twistAngle);}

      // If the right back Module is in the front
      if ((Math.abs(SwerveModule.closestAngle(direction, 315.0))) >= 90.0) {rightBackModule.setDirection(direction + twistAngle);}
      // if it's in the back
      else {rightBackModule.setDirection(direction - twistAngle);}
  
      leftFrontModule.setSpeed(translatePower);
      leftBackModule.setSpeed(translatePower);
      rightFrontModule.setSpeed(translatePower);
      rightBackModule.setSpeed(translatePower);
  }
  public void swerveMove(double direction, double translatePower, double twistPower) {
    translatePower = SwerveModule.deadzone(translatePower, Constants.SwerveConstants.MOVEMENT_SPEED_DEADZONE);
    twistPower = SwerveModule.deadzone(twistPower, Constants.SwerveConstants.TWIST_DEADZONE);
    if ((translatePower == 0) && (twistPower != 0)){inplaceTurn(twistPower);}
    else {translateTurn(direction, translatePower, twistPower);}
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run#
  }
}
