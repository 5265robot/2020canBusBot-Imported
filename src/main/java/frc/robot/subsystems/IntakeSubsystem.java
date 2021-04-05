/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final Spark m_intakeSpark = new Spark(IntakeConstants.kIntakeMotorPWM);
  private final Spark m_lowerSpark = new Spark(IntakeConstants.kLowerMotorPWM);
  private final Spark m_upperSpark = new Spark(IntakeConstants.kUpperMotorPWM);

  public IntakeSubsystem() {
    // initialization methods here
  }

  // on/off switch for the intake
  public void intakeOn(double power, boolean state){
    if (!state) {
      m_intakeSpark.set(power);}
    else m_intakeSpark.set(0.0);
    Constants.currentIntakeState = !state;
  }

  // powers the upper conveyor belt
  public void upperOn(double power){
    m_upperSpark.set(power);
  }
  // powers the lower conveyor belt
  public void lowerOn(double power){
    m_lowerSpark.set(power);
  }
  // powers both conveyor belts,
  // giving the upper belt a bit more power
  public void upperAndLowerOn(double power){
    // can we put a timeout here
    upperOn(power*IntakeConstants.kUpperLowerRatio);
    lowerOn(power);
  }
  // turns off the conveyor belts
  public void upperAndLowerOff(){
    upperOn(0.0);
    lowerOn(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
