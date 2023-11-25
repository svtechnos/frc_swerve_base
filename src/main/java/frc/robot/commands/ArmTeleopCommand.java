package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class ArmTeleopCommand extends CommandBase {
  
  public void armTeleopPeriodic() {
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

