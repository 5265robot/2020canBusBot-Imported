/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new ArmPIDSubsystem.
   */

  private final VictorSP m_arm = new VictorSP(ArmConstants.kArmMotorPWM);
  private final Spark m_winch = new Spark(ArmConstants.kWinchPWM);
  private final Encoder m_encoder = new Encoder(ArmConstants.kchannelA, ArmConstants.kchannelB);
  
  private final DoubleSolenoid m_pinchSolenoid =
    new DoubleSolenoid(ArmConstants.kPinchSolenoidPortA, ArmConstants.kPinchSolenoidPortB);
  private final DoubleSolenoid m_elbowSolenoid =
    new DoubleSolenoid(ArmConstants.kArmSolenoidPortA, ArmConstants.kArmSolenoidPortB);


  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kEncoderP, ArmConstants.kEncoderI, ArmConstants.kEncoderD));
        m_encoder.setDistancePerPulse((ArmConstants.reduction*ArmConstants.countsPerRevolution)/360);
        getController().setTolerance(ArmConstants.kSetTolerance);
    //    m_encoder.setReverseDirection(true);
        this.setSetpoint(ArmConstants.kSetPoint);
  }

//  public void testArm(){
  //  this.setSetpoint(ArmConstants.kSetPoint);
 // }

  public void armDown(){
    if (!Constants.armExtended) {
      m_arm.set(0.1);
    }
    else {
      m_arm.set(0.0);
    }
    Constants.armExtended = !Constants.armExtended;
  }

  public void testArmOff(){
    this.disable();
    m_encoder.reset();
    m_arm.set(0.0);
  }

  // turns the winch on/off
  public void winchOn(double power) {
    if (!Constants.winchState) {
      m_winch.set(power);
    }
    else {
      m_winch.set(0.0); 
    }
    Constants.winchState = !Constants.winchState;
  } // end winchOn

  public void firePinch(){
    // pinch solenoid changes color but does not fire by command
    DoubleSolenoid.Value value = m_pinchSolenoid.get();
    if (value == DoubleSolenoid.Value.kReverse){
      System.out.println("pinch solenoid fired forward");
      m_pinchSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else {
      System.out.println("pinch solenoid fired reverse");
      m_pinchSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
  public void fireElbow(){
    // forward adducts the elbow (brings it into the robot)
    // reverse abducts the elbow (pushes it out of the robot)
    DoubleSolenoid.Value value = m_elbowSolenoid.get();
    if (value == DoubleSolenoid.Value.kReverse){
      System.out.println("elbox solenoid fired forward");
      m_elbowSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else {
      System.out.println("elbox solenoid fired reverse");
      m_elbowSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_arm.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.get();
  }

public void raiseArmUp() {
	this.enable();
  System.out.println("output for arm is on");
}
}
