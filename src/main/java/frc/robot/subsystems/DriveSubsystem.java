/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // drn --
  // motors, motor groups, and drive
  private final CANSparkMax m_zeroWheel = 
    new CANSparkMax(DriveConstants.kLeftMotor00CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_oneWheel = 
    new CANSparkMax(DriveConstants.kLeftMotor01CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_twoWheel = 
    new CANSparkMax(DriveConstants.kRightMotor02CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_threeWheel = 
    new CANSparkMax(DriveConstants.kRightMotor03CanBusID, MotorType.kBrushless);
  
  private final SpeedControllerGroup m_leftMotors =
    new SpeedControllerGroup(m_zeroWheel, m_oneWheel);
  private final SpeedControllerGroup m_rightMotors =
    new SpeedControllerGroup(m_twoWheel, m_threeWheel);
  
  private final DifferentialDrive m_drive =
    new DifferentialDrive(m_leftMotors, m_rightMotors);

  private AHRS m_ahrs;
   // new AHRS(SPI.Port.kMXP);

  // Set up odometry class
  private DifferentialDriveOdometry m_odometry;
  public Pose2d m_pose;

  // Set up field diagram
  private final Field2d m_field2D = new Field2d();
  
  public DriveSubsystem() {
    // pairing the motors
    m_oneWheel.follow(m_zeroWheel);
    m_twoWheel.follow(m_threeWheel);
    //m_ahrs.     need to set up gyro
    //
    m_drive.setDeadband(0.15);
    m_drive.setMaxOutput(DriveConstants.kMaxSpeed);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    m_odometry = new DifferentialDriveOdometry(m_ahrs.getRotation2d());
  }

  // 
  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(-fwd*Math.abs(fwd), rot);
  }

  public void curveDrive(double fwd, double rot, boolean qT){
    m_drive.curvatureDrive(fwd, rot, qT);
  }

  // straight driving... needs gyro added
  public void simpleDrive(double kpower) {
    m_drive.arcadeDrive(kpower, 0.0);
  }

  // to drop maximum speed for delicate motion
  public void setMax(double maxOutput){
    m_drive.setMaxOutput(maxOutput);
  }

  // turns the to half and then back on
  public void halfPower() {
    if (!Constants.powerState) {
      m_drive.setMaxOutput(DriveConstants.kSlowMaxSpeed);
    }
    else {
      m_drive.setMaxOutput(DriveConstants.kMaxSpeed); 
    }
    Constants.powerState = !Constants.powerState;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    m_drive.feed();
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftMotors.get(),m_rightMotors.get());
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    // resetEncoders();
    // m_ahrs.reset();
    m_odometry.resetPosition(pose, m_ahrs.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
