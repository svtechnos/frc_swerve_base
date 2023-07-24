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
    rightFrontModule.setDirection(225.0);
    rightBackModule.setDirection(135.0);
  }
  public void inplaceTurn(double power){
    leftFrontModule.setDirection(315.0);
    leftBackModule.setDirection(45.0);
    rightFrontModule.setDirection(225.0);
    rightBackModule.setDirection(135.0);
    leftFrontModule.setSpeed(power);
    leftBackModule.setSpeed(power);
    rightFrontModule.setSpeed(power);
    rightBackModule.setSpeed(power);
  }
  public void swerveMove(double direction, double translatePower, double twistPower,double modifier) {
    translatePower = SwerveModule.deadzone(translatePower, Constants.SwerveConstants.MOVEMENT_SPEED_DEADZONE);
    twistPower = SwerveModule.deadzone(twistPower, Constants.SwerveConstants.TWIST_DEADZONE);
    if ((translatePower == 0) && (twistPower != 0)){inplaceTurn(twistPower*modifier);}
    else {translateTurn(direction, translatePower, SwerveModule.deadzone(twistPower,0.2));}
  }
  
  public double angleCalculator(double direction, double translatePower, double twistPower, double twistVectorDirection){
    double yval= ((translatePower * Math.sin(direction*2.0*Math.PI/180.0)) + (twistPower*Math.cos(twistVectorDirection*Math.PI/180.0)));
    double xval= ((translatePower * Math.sin(direction*2.0*Math.PI/180.0)) + (twistPower*Math.cos(twistVectorDirection*Math.PI/180.0)));
    return Math.atan2(yval,xval);
  }

  public double speedCalculator(double direction, double translatePower, double twistPower, double twistVectorDirection){
    double yval= ((translatePower * Math.sin(direction*2.0*Math.PI/180.0)) + (twistPower*Math.cos(twistVectorDirection*Math.PI/180.0)));
    double xval= ((translatePower * Math.sin(direction*2.0*Math.PI/180.0)) + (twistPower*Math.cos(twistVectorDirection*Math.PI/180.0)));
    return Math.sqrt(yval*yval + xval*xval);
  }


  public void translateTurn(double direction, double translatePower, double twistPower){
    double twistAngle=twistPower*-45;
    direction = direction%360;
    /* 
    leftFrontModule.setDirection(angleCalculator(direction, translatePower, twistPower, 315));
    leftBackModule.setDirection(angleCalculator(direction, translatePower, twistPower, 45));
    rightFrontModule.setDirection(angleCalculator(direction, translatePower, twistPower, 225));
    rightBackModule.setDirection(angleCalculator(direction, translatePower, twistPower, 135));
    
    leftFrontModule.setSpeed(speedCalculator(direction, translatePower, twistPower, 315));
    leftBackModule.setSpeed(speedCalculator(direction, translatePower, twistPower, 45));
    rightFrontModule.setSpeed(speedCalculator(direction, translatePower, twistPower, 225));
    rightBackModule.setSpeed(speedCalculator(direction, translatePower, twistPower, 135));
    */
    if ((Math.abs(SwerveModule.closestAngle(direction, 0))) <= 45) 
    {
      //System.out.println("Front facing");
      leftFrontModule.setDirection((direction + twistAngle)%360);
      rightFrontModule.setDirection((direction + twistAngle)%360);
      leftBackModule.setDirection((direction - twistAngle)%360);
      rightBackModule.setDirection((direction - twistAngle)%360);
    }
    else if ((Math.abs(SwerveModule.closestAngle(direction, 90))) <= 45) 
    {
      //System.out.println("left facing");
      leftFrontModule.setDirection((direction + twistAngle)%360);
      rightFrontModule.setDirection((direction - twistAngle)%360);
      leftBackModule.setDirection((direction + twistAngle)%360);
      rightBackModule.setDirection((direction - twistAngle)%360);

    }else if ((Math.abs(SwerveModule.closestAngle(direction, 180))) <= 45) 
    {
      //System.out.println("back facing");
      leftFrontModule.setDirection((direction - twistAngle)%360);
      rightFrontModule.setDirection((direction - twistAngle)%360);
      leftBackModule.setDirection((direction + twistAngle)%360);
      rightBackModule.setDirection((direction + twistAngle)%360);
    }else if ((Math.abs(SwerveModule.closestAngle(direction, 270))) <= 45) 
    {
      //System.out.println("right facing");
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
  @Override
  public void periodic() {}
}
