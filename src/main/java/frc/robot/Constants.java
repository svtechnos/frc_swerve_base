// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class DeviceIDs {
    public static final int LEFT_FRONT_DRIVE_ID=1;
    public static final int LEFT_FRONT_TURN_ID=2;
    public static final int LEFT_FRONT_TURN_ENCODER_ID=21;
    public static final int RIGHT_FRONT_DRIVE_ID=3;
    public static final int RIGHT_FRONT_TURN_ID=4;
    public static final int RIGHT_FRONT_TURN_ENCODER_ID=22;
    public static final int LEFT_BACK_DRIVE_ID=5;
    public static final int LEFT_BACK_TURN_ID=6;
    public static final int LEFT_BACK_TURN_ENCODER_ID=23;
    public static final int RIGHT_BACK_DRIVE_ID=7;
    public static final int RIGHT_BACK_TURN_ID=8;
    public static final int RIGHT_BACK_TURN_ENCODER_ID=24;
    public static final int GYRO_DEVICE_ID=10;
  }
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int ARM_CONTROLLER_PORT = 1;
  }
  public static class Directions {
    public static final int FORWARD = 0;
    public static final int LEFT = 90;
    public static final int BACKWARD = 180;
    public static final int RIGHT = 270;
  }
  public static class ChargeStationConstants {
    public static final double YAW_P = 0.01;
    public static final double TILT_P = 0.02;
    public static final double DEADZONE = 3;
  }
}