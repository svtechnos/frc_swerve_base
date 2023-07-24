// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.sensors.Pigeon2;


public class SwerveCoordinator extends SubsystemBase {
  public SwerveModule leftFrontModule;
  public SwerveModule leftBackModule;
  public SwerveModule rightFrontModule;
  public SwerveModule rightBackModule;
  public SwerveDriveKinematics m_kinematics;
    
  // Gyro
  public static Pigeon2 GYRO;

  /** Creates a new SwerveCoordinator. */
  public SwerveCoordinator(SwerveModule leftFrontModule, SwerveModule leftBackModule, SwerveModule rightFrontModule, SwerveModule rightBackModule) {
    this.leftFrontModule = leftFrontModule;
    this.leftBackModule = leftBackModule;
    this.rightFrontModule = rightFrontModule;
    this.rightBackModule = rightBackModule;
    GYRO = new Pigeon2(Constants.DeviceIDs.GYRO_DEVICE_ID, "rio");
    m_kinematics= new SwerveDriveKinematics(
     Constants.SwerveConstants.m_frontLeftLocation, Constants.SwerveConstants.m_frontRightLocation, Constants.SwerveConstants.m_backLeftLocation, Constants.SwerveConstants.m_backRightLocation
    );
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
  public void swerveMove(double xspeed, double yspeed, double twistPower, double modifier) {
    //ChassisSpeeds speeds = new ChassisSpeeds(xspeed, yspeed, twistPower);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, twistPower, Rotation2d.fromDegrees(GYRO.getYaw()));

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];
    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];
    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];
    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    leftFrontModule.move_module(frontLeft);
    rightFrontModule.move_module(frontRight);
    leftBackModule.move_module(backLeft);
    rightBackModule.move_module(backRight);
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

  public double currentAngle(){
    return (((turnEncoder.getSelectedSensorPosition())*(360.0/4096.0))-turnEncoderOffset)%360;
  }

  /*public void translateTurn(double direction, double translatePower, double twistPower){
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
  }*/
  @Override
  public void periodic() {}
}
