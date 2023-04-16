// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Offsets {
    public static final double LEFT_FRONT_TURN_ENCODER_OFFSET = 23.2929;
    public static final double LEFT_BACK_TURN_ENCODER_OFFSET = 321.3847;
    public static final double RIGHT_FRONT_TURN_ENCODER_OFFSET = 4.9218;
    public static final double RIGHT_BACK_TURN_ENCODER_OFFSET = 242.2714;
  }
  public static class DeviceIDs {
    public static final int LEFT_FRONT_DRIVE_ID=5;
    public static final int LEFT_FRONT_TURN_ID=7;
    public static final int LEFT_FRONT_TURN_ENCODER_ID=24;
    public static final int RIGHT_FRONT_DRIVE_ID=3;
    public static final int RIGHT_FRONT_TURN_ID=8;
    public static final int RIGHT_FRONT_TURN_ENCODER_ID=21;
    public static final int LEFT_BACK_DRIVE_ID=1;
    public static final int LEFT_BACK_TURN_ID=4;
    public static final int LEFT_BACK_TURN_ENCODER_ID=23;
    public static final int RIGHT_BACK_DRIVE_ID=6;
    public static final int RIGHT_BACK_TURN_ID=2;
    public static final int RIGHT_BACK_TURN_ENCODER_ID=22;
    public static final int GYRO_DEVICE_ID=11;
  }
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int ARM_CONTROLLER_PORT = 1;
  }
  public static class Directions {
    public static final double FORWARD = 0;
    public static final double LEFT = 90;
    public static final double BACKWARD = 180;
    public static final double RIGHT = 270;
  }
  public static class ChargeStationConstants {
    public static final double YAW_P = 0.01;
    public static final double TILT_P = 0.01;
    public static final double DEADZONE = 1;
    public static final double START_SPEED = 0.15;
    public static final double START_CLIMB_ANGLE = 10;
  }
  public static class SwerveConstants {
    public static final double MODULE_ROTATION_P = 0.005;
    public static final double TURN_ANGLE_DEADZONE = 2;
    public static final double MOVEMENT_SPEED_DEADZONE = 0.05;
    public static final double CLIP_SPEED = 0.5;
    public static final double DRIVE_MOTOR_RAMP_RATE = 0.3;
    public static final double TURN_MOTOR_RAMP_RATE = 0.3;
    public static final double TWIST_DEADZONE = 0.08;
  }
  public static class DirectionDriveConstants{
    public static final double YAW_P=0.01;
  }
}