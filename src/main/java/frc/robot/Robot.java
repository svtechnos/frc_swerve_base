// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;
  private RobotContainer m_robotContainer;
    //Arm fields and constants start
    private static final int deviceID = 43;
    private static final int deviceID2 = 42;
    private static final int deviceID3 = 41;
    private static final int deviceID4 = 40;
    private CANSparkMax m_motor, m_motor2, m_motor3, m_motor4;
    private SparkMaxPIDController m_pidController, m_pidController2, m_pidController3, m_pidController4;
    private RelativeEncoder m_encoder, m_encoder2, m_encoder3, m_encoder4;
    private Joystick m_joystick;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, lastButtonPressed1, lastButtonPressed2, lastButtonPressed3, oldValue1 = -1000, oldValue2 = -1000, oldValue3 = -1000, run;
    public boolean presetState;
    public double setPoint1 = 0;
    public double setPoint2 = 0;
    public double setPoint3 = 0;
    public double gain1 = 42;
    public double gain2 = 22;
    public double gain3 = 15;
    //Arm fields and constants end
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(0);
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(deviceID2, MotorType.kBrushless);
    m_motor3 = new CANSparkMax(deviceID3, MotorType.kBrushless);
    m_motor4 = new CANSparkMax(deviceID4, MotorType.kBrushless);
    //CameraServer.startAutomaticCapture(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {m_autonomousCommand.schedule();}
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_teleopCommand = m_robotContainer.getTeleopCommand();
    if (m_teleopCommand != null) {m_teleopCommand.schedule();}
    if (m_autonomousCommand != null) {m_autonomousCommand.cancel();}
    armInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {armTeleopPeriodic();}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  private void armInit() {
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
    m_joystick = new Joystick(Constants.OperatorConstants.kArmControllerPort);
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

  private void armTeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to controller

    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController2.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController2.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController2.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController2.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    if((p != kP)) { m_pidController2.setP(p); kP = p; }
    if((i != kI)) { m_pidController2.setI(i); kI = i; }
    if((d != kD)) { m_pidController2.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController2.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController2.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController2.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController3.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController3.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController3.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController3.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    if((p != kP)) { m_pidController3.setP(p); kP = p; }
    if((i != kI)) { m_pidController3.setI(i); kI = i; }
    if((d != kD)) { m_pidController3.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController3.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController3.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController3.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if((maxV != maxVel)) { m_pidController4.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController4.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController4.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController4.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    if((p != kP)) { m_pidController4.setP(p); kP = p; }
    if((i != kI)) { m_pidController4.setI(i); kI = i; }
    if((d != kD)) { m_pidController4.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController4.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController4.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController4.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
 
 
    double processVariable, processVariable2, processVariable3, processVariable4;
    boolean mode = SmartDashboard.getBoolean("Mode", false);

    //double setPoint1 = SmartDashboard.getNumber("Set Position", 0);
    //double setPoint2 = SmartDashboard.getNumber("Set Position2", 0);
    //double setPoint3 = SmartDashboard.getNumber("Set Position3", 0);
    double IntakeSetVelocity = SmartDashboard.getNumber("Intake Set Velocity", 0);
    
    //setPoint = SmartDashboard.gZetNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */

      if (run<1000){
        run++;
      }
      if (run<200){
        if (run<50){
          m_pidController.setReference(5, CANSparkMax.ControlType.kSmartMotion);
          m_pidController2.setReference(-5, CANSparkMax.ControlType.kSmartMotion);
          m_pidController3.setReference(5, CANSparkMax.ControlType.kSmartMotion);
        }
        else{
          m_pidController.setReference(-100, CANSparkMax.ControlType.kSmartMotion);
          m_pidController2.setReference(100, CANSparkMax.ControlType.kSmartMotion);
          m_pidController3.setReference(-100, CANSparkMax.ControlType.kSmartMotion);
        }
      }
      else if (run==200){
        
    m_encoder.setPosition(0);
    m_encoder2.setPosition(0);
    m_encoder3.setPosition(0);
    m_encoder4.setPosition(0);
      }
      else{
      double currentValue1 = ((-m_joystick.getRawAxis(0)+1)*gain1);
      double currentValue2 = ((-m_joystick.getRawAxis(1)+1)*-gain2);
      double currentValue3 = ((-m_joystick.getRawAxis(2)+1)*gain3);



      if (m_joystick.getRawButtonPressed(1)){
        presetState = true;
        lastButtonPressed1 = 1; 
        lastButtonPressed2 = 1;
        lastButtonPressed3 = 1;
      }

      if (m_joystick.getRawButtonPressed(2)){
        presetState = true;
        lastButtonPressed1 = 2; 
        lastButtonPressed2 = 2;
        lastButtonPressed3 = 2;
      }

      if (m_joystick.getRawButtonPressed(3)){
        //presetState = true;
        lastButtonPressed1 = 3; 
        lastButtonPressed2 = 3;
        lastButtonPressed3 = 3;
      }
      if (m_joystick.getRawButtonPressed(4)){
        //presetState = true;
        lastButtonPressed1 = 4; 
        lastButtonPressed2 = 4;
        lastButtonPressed3 = 4;
      }

      double dead = 0.03;
      double dead2 = 0.2;
      double dead3 = 0.3;
      if (Math.abs((currentValue1 - oldValue1 )) > gain1*dead) {
        oldValue1 = currentValue1;
        setPoint1 = currentValue1;
        System.out.println("Arash");
        lastButtonPressed1 = 0;
      }
      //else{
        //setPoint1 = oldValue1;
      //}

      if (Math.abs((currentValue2 - oldValue2 )) > gain2*dead) {
        oldValue2 = currentValue2;
        setPoint2 = currentValue2;
        lastButtonPressed2 = 0;
      }

  
      //else{
        //setPoint2 = oldValue2;
      //}

      if (Math.abs((currentValue3 - oldValue3 )) > gain3*dead) {
        oldValue3 = currentValue3;
        setPoint3 = currentValue3;
        lastButtonPressed3 = 0;
      }
      //else{
        //setPoint3 = oldValue3;
      //}
    
      //Preset 1

      if (lastButtonPressed1 == 1){
        setPoint1 = -10;
      }

      if (lastButtonPressed2 == 1){
        setPoint2 = 100;
      }

      if (lastButtonPressed3 == 1){
        setPoint3 = 14.65;
      }

      //Preset 2^
      if (lastButtonPressed1 == 2){
        setPoint1 = 80;
      }

      if (lastButtonPressed2 == 2){
        setPoint2 = -20.441;
      }

      if (lastButtonPressed3 == 2){
        setPoint3 = 14.78; 
      }

      //Preset 3
      if (lastButtonPressed1 == 3){
        setPoint1 = 80;
      }

      if (lastButtonPressed2 == 3){
        setPoint2 = -20.441;
      }

      if (lastButtonPressed3 == 3){
        setPoint3 = 0;
      }

      //Preset 4
      if (lastButtonPressed1 == 4){
        setPoint1 = 0;
      }

      if (lastButtonPressed2 == 4){
        setPoint2 = 1;
      }

      if (lastButtonPressed3 == 4){
        setPoint3 = -9.5;
      }


      System.out.print("1 ");
      System.out.println(setPoint1);
      System.out.print("2 ");
      System.out.println(setPoint2);
      System.out.print("3 ");
      System.out.println(setPoint3);

      //System.out.println(lastButtonPressed1);
      //System.out.println(lastButtonPressed2);
      //System.out.println(lastButtonPressed3);
        
       // if ((setPoint > (m_joystick.getRawAxis(1)*15)-0.1) && (setPoint < (m_joystick.getRawAxis(1)*15)+0.1)){
          //presetState = false;
        //}
      

      if (m_joystick.getRawButton(5)){
        //IntakeSetVelocity = -2000;
        IntakeSetVelocity = 6000;
        //SmartDashboard.getNumber("Intake Set Velocity", 6000);
      }
      else if (m_joystick.getRawButton(6)){
        IntakeSetVelocity = -6000;
        //(0-SmartDashboard.getNumber("Intake Set Velocity", -6000));
        //IntakeSetVelocity = 2000;
      }
      else{
        IntakeSetVelocity = 0;
      }
      //System.out.println(m_joystick.getRawAxis(1)*15);

      m_pidController.setReference(setPoint1, CANSparkMax.ControlType.kSmartMotion);
      m_pidController2.setReference(setPoint2, CANSparkMax.ControlType.kSmartMotion);
      m_pidController3.setReference(setPoint3, CANSparkMax.ControlType.kSmartMotion);
      m_pidController4.setReference(IntakeSetVelocity, CANSparkMax.ControlType.kSmartVelocity); //smart velocity or velocty


      processVariable = m_encoder.getPosition();
      processVariable2 = m_encoder2.getPosition();
      processVariable3 = m_encoder3.getPosition();
      processVariable4 = m_encoder4.getPosition();
    
    
    SmartDashboard.putNumber("Setpoint", setPoint1);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    
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
        //SmartDashboard.putNumber("Set Position", 0);
        //SmartDashboard.putNumber("Set Velocity", 0);
    
        // button to toggle between velocity and smart motion modes
        //SmartDashboard.putBoolean("Mode", true);
    }
  }
}
