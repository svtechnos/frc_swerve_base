package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;
  private RobotContainer m_robotContainer;
    //Arm fields and constants start

    private Joystick m_joystick;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, lastButtonPressed1, lastButtonPressed2, lastButtonPressed3, oldValue1 = -1000, oldValue2 = -1000, oldValue3 = -1000, run;
    public boolean presetState;
    public double setPoint1 = 0;
    public double setPoint2 = 0;
    public double setPoint3 = 0;
    public double gain1 = 42;
    public double gain2 = 22;
    public double gain3 = 15;
   
  public void armInit() {
    // initialize motor

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor2.setIdleMode(IdleMode.kBrake);
    m_motor3.setIdleMode(IdleMode.kBrake);
    m_motor4.setIdleMode(IdleMode.kBrake);


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     
    m_motor.restoreFactoryDefaults();
    m_motor2.restoreFactoryDefaults();
    m_motor3.restoreFactoryDefaults();
    m_motor4.restoreFactoryDefaults();
    **/

    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_pidController2 = m_motor2.getPIDController();
    m_pidController3 = m_motor3.getPIDController();
    m_pidController4 = m_motor4.getPIDController();
    m_encoder = m_motor.getEncoder();
    m_encoder2 = m_motor2.getEncoder();
    m_encoder3 = m_motor3.getEncoder();
    m_encoder4 = m_motor4.getEncoder();
    m_joystick = new Joystick(Constants.OperatorConstants.ARM_CONTROLLER_PORT);
    presetState = false;
    lastButtonPressed1 = 0; //0 is like the null value
    lastButtonPressed2 = 0;
    lastButtonPressed3 = 0;

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;




    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

    
    m_pidController3.setP(kP);
    m_pidController3.setI(kI);
    m_pidController3.setD(kD);
    m_pidController3.setIZone(kIz);
    m_pidController3.setFF(kFF);
    m_pidController3.setOutputRange(kMinOutput, kMaxOutput);

        
    m_pidController4.setP(kP);
    m_pidController4.setI(kI);
    m_pidController4.setD(kD);
    m_pidController4.setIZone(kIz);
    m_pidController4.setFF(kFF);
    m_pidController4.setOutputRange(kMinOutput, kMaxOutput);


    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_pidController2.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController2.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController2.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController2.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_pidController3.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController3.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController3.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController3.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    
    m_pidController4.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController4.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController4.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController4.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
    SmartDashboard.putNumber("Setpoint", 0);
    SmartDashboard.putNumber("Intake Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);

       /** 
    m_pidController.setReference(-10, CANSparkMax.ControlType.kSmartMotion);
    m_pidController2.setReference(10, CANSparkMax.ControlType.kSmartMotion);
    m_pidController3.setReference(-10, CANSparkMax.ControlType.kSmartMotion);
    try {   
       Thread.sleep(100);
      }
    catch(Exception e){

    }
    
    m_pidController.setReference(-100, CANSparkMax.ControlType.kSmartMotion);
    m_pidController2.setReference(100, CANSparkMax.ControlType.kSmartMotion);
    m_pidController3.setReference(-100, CANSparkMax.ControlType.kSmartMotion);
    try {   
       Thread.sleep(500);
      }
    catch(Exception e){

    }

    m_encoder.setPosition(0);
    m_encoder2.setPosition(0);
    m_encoder3.setPosition(0);
    m_encoder4.setPosition(0);
    */
    run = 0;
    

  }

}

 
