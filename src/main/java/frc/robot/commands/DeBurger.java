// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DeBurger extends CommandBase {
  SwerveDrive swerveDrive;
  Joystick joystick;
  /**
   * Debug command, used to print values, may get absorbed into monitoring system once it is made
   * @param swerveDrive
   * @param joystick
   */
  public DeBurger(SwerveDrive swerveDrive, Joystick joystick) {
    this.swerveDrive = swerveDrive;
    this.joystick = joystick;
    addRequirements(swerveDrive);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    if(joystick.getRawButton(7)){
      System.out.println("currentAngle of leftFront: "+SwerveDrive.LEFT_FRONT_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0);
      System.out.println("currentAngle of leftBack: "+SwerveDrive.LEFT_BACK_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0);
      System.out.println("currentAngle of rightFront: "+SwerveDrive.RIGHT_FRONT_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0);
      System.out.println("currentAngle of rightBack: "+SwerveDrive.RIGHT_BACK_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0);
    }
    else if(joystick.getRawButton(8)){
      System.out.println("Gyro Yaw: "+SwerveDrive.GYRO.getYaw());
      System.out.println("Gyro Pitch: "+SwerveDrive.GYRO.getPitch());
      System.out.println("Gyro Roll: "+SwerveDrive.GYRO.getRoll());
    }
    else if(joystick.getRawButton(9)){
      System.out.println("currentAngle of leftFront: "+((SwerveDrive.LEFT_FRONT_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0)-Constants.Offsets.LEFT_FRONT_TURN_ENCODER_OFFSET)%360);
      System.out.println("currentAngle of leftBack: "+((SwerveDrive.LEFT_BACK_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0)-Constants.Offsets.LEFT_BACK_TURN_ENCODER_OFFSET)%360);
      System.out.println("currentAngle of rightFront: "+((SwerveDrive.RIGHT_FRONT_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0)-Constants.Offsets.RIGHT_FRONT_TURN_ENCODER_OFFSET)%360);
      System.out.println("currentAngle of rightBack: "+((SwerveDrive.RIGHT_BACK_TURN_ENCODER.getSelectedSensorPosition()*360.0/4096.0)-Constants.Offsets.RIGHT_BACK_TURN_ENCODER_OFFSET)%360);
    }
    else if(joystick.getRawButton(10)){
      System.out.println("10");
    }
    else if(joystick.getRawButton(11)){
      System.out.println("11");
    }
    else if(joystick.getRawButton(12)){
      System.out.println("12");
    }
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
