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
    // bitofissue
    rightFrontModule.setDirection(-45.0);
    rightBackModule.setDirection(-135.0);
  }
  public void inplaceTurn(double power){
    leftFrontModule.setDirection(315.0);
    leftBackModule.setDirection(45.0);
    // bitofissue
    rightFrontModule.setDirection(225.0);
    rightBackModule.setDirection(135.0);
    //might have to fix stuff
    leftFrontModule.setSpeed(power);
    leftBackModule.setSpeed(power);
    rightFrontModule.setSpeed(power);
    rightBackModule.setSpeed(power);
  }
  /*public void translateTurn(double direction, double translatePower, double twistPower){
    System.out.println("directionNEW: "+direction);
    if(false){
        // turnAngle goes from 45 to -45
        double twistAngle = twistPower * 45.0;
        twistAngle=30;
        direction=direction%360;
        // If the left front Module is in the front
        //og:135
        if ((Math.abs(SwerveModule.closestAngle(direction, 315.0))) >= 90.0) {System.out.println("leftfront=front");leftFrontModule.setDirection((direction + twistAngle)%360);}
        // if it's in the back
        else {System.out.println("leftfront=back");leftFrontModule.setDirection((direction - twistAngle)%360);}

        //og: 225
        // If the left back Module is in the front
        if ((Math.abs(SwerveModule.closestAngle(direction, 225.0))) > 90.0) {System.out.println("leftback=front");leftBackModule.setDirection((direction + twistAngle)%360);}
        // if it's in the back
        else {leftBackModule.setDirection((direction - twistAngle)%360);System.out.println("leftback=back");}

        //og:45
        // If the right front Module is in the front
        if ((Math.abs(SwerveModule.closestAngle(direction, 45.0))) > 90.0) {System.out.println("rightfront=front");rightFrontModule.setDirection((direction + twistAngle)%360);}
        // if it's in the back
        else {rightFrontModule.setDirection((direction - twistAngle)%360);System.out.println("rightfront=back");}

        //og:315
        // If the right back Module is in the front
        if ((Math.abs(SwerveModule.closestAngle(direction, 135.0))) >= 90.0) {System.out.println("rightback=front");rightBackModule.setDirection((direction + twistAngle)%360);}
        // if it's in the back
        else {rightBackModule.setDirection((direction - twistAngle)%360);System.out.println("rightback=back");}
    }
    else{

      // front facing {lf,rf}
      // back facing {lb,rb}
      double sub=0;
      double sideOne = (direction+sub)%360;
      double sideTwo = (direction-sub)%360;
      leftFrontModule.setDirection(sideOne);
      leftBackModule.setDirection(sideTwo);
      rightBackModule.setDirection(sideTwo);
      rightFrontModule.setDirection(sideOne);
    }
      double gain = 1;
      leftFrontModule.setSpeed(gain*translatePower);
      leftBackModule.setSpeed(gain*translatePower); 
      rightFrontModule.setSpeed(translatePower/gain);
      rightBackModule.setSpeed(translatePower/gain);
  }*/
  public void swerveMove(double direction, double translatePower, double twistPower) {
    translatePower = SwerveModule.deadzone(translatePower, Constants.SwerveConstants.MOVEMENT_SPEED_DEADZONE);
    twistPower = SwerveModule.deadzone(twistPower, Constants.SwerveConstants.TWIST_DEADZONE);
    if ((translatePower == 0) && (twistPower != 0)){inplaceTurn(twistPower/2);}
    else {translateTurn(direction, translatePower, SwerveModule.deadzone(twistPower,0.3));}
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run#
  }
  public void translateTurn(double direction, double translatePower, double twistPower){
    // turnAngle goes from 45 to -45
    double twistAngle = twistPower * -45.0;
    //twistAngle=30;
    direction=direction%360;
    // If the left front Module is in the front
    //og:135
    if ((Math.abs(SwerveModule.closestAngle(direction, 0))) <= 45) 
    {
      System.out.println("Front facing");
      leftFrontModule.setDirection((direction + twistAngle)%360);
      rightFrontModule.setDirection((direction + twistAngle)%360);
      leftBackModule.setDirection((direction - twistAngle)%360);
      rightBackModule.setDirection((direction - twistAngle)%360);
    }
    // if it's in the back
    else if ((Math.abs(SwerveModule.closestAngle(direction, 90))) <= 45) 
    {
      System.out.println("left facing");
      leftFrontModule.setDirection((direction + twistAngle)%360);
      rightFrontModule.setDirection((direction - twistAngle)%360);
      leftBackModule.setDirection((direction + twistAngle)%360);
      rightBackModule.setDirection((direction - twistAngle)%360);

    }else if ((Math.abs(SwerveModule.closestAngle(direction, 180))) <= 45) 
    {
      System.out.println("back facing");
      leftFrontModule.setDirection((direction - twistAngle)%360);
      rightFrontModule.setDirection((direction - twistAngle)%360);
      leftBackModule.setDirection((direction + twistAngle)%360);
      rightBackModule.setDirection((direction + twistAngle)%360);
    }else if ((Math.abs(SwerveModule.closestAngle(direction, 270))) <= 45) 
    {
      System.out.println("right facing");
      leftFrontModule.setDirection((direction - twistAngle)%360);
      rightFrontModule.setDirection((direction + twistAngle)%360);
      leftBackModule.setDirection((direction - twistAngle)%360);
      rightBackModule.setDirection((direction + twistAngle)%360);
    }
      double gain = 1;
      leftFrontModule.setSpeed(gain*translatePower);
      leftBackModule.setSpeed(gain*translatePower);
      rightFrontModule.setSpeed(translatePower/gain);
      rightBackModule.setSpeed(translatePower/gain);
  }
}
