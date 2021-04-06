/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // auto for Ramsete
    public static final class rAutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.4; // was 0.8
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.30; // was 0.8
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
    }
    // auto trajectory
    public static final class rTrajectoryConstants {
        private static final double kWheelDiameterMeter = 0.070;
        public static final double kWheelDiameter = kWheelDiameterMeter;
        public static final List<Translation2d> pathApartAPoints = List.of(
                new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+22.0), Units.inchesToMeters(+10.0))
                );
        public static final Pose2d pathApartAstartPose =
                new Pose2d(Units.inchesToMeters(-13.5), Units.inchesToMeters(+4.5), new Rotation2d(Units.degreesToRadians(-90)));
        public static final Pose2d pathApartAendPose = 
                new Pose2d(Units.inchesToMeters(+11.0), Units.inchesToMeters(-3.1), new Rotation2d(Units.degreesToRadians(-90)));
          
    }


    // on / off switches
    public static boolean currentIntakeState = false;
    public static boolean cameraState = false;
    public static boolean armExtended = false;
    public static boolean winchState = false;
	public static boolean powerState = false;
	
    // XBox mappings
    /*  kBumperLeft(5)  --> half speed driving
        kBumperRight(6) --> extend elbow
        kStickLeft(9)   --> conveyor empty
        kStickRight(10) --> 
        kA(1)           --> intake
        kB(2)           --> conveyor one ball pulse
        kX(3)           --> disable climbarm
        kY(4)           --> switch camera
        kBack(7)        --> intake reverse
        kStart(8)       --> 
    */
    public static int kIntakeButton = XboxController.Button.kA.value;   
    public static int kConveyorPulseButton = XboxController.Button.kB.value;   
    public static int kClimbOffButton = XboxController.Button.kX.value;
    public static int kSwitchCameraButton = XboxController.Button.kY.value;   
    public static int kSlowDown = XboxController.Button.kBumperLeft.value;
    public static int kElbowExtend = XboxController.Button.kBumperRight.value;
	public static int kIntakeReverseButton = XboxController.Button.kBack.value;
    public static int kConveyorEmptyButton = XboxController.Button.kStickLeft.value;
    
    // thrustmaster buttons
	public static int kThrustPulseButton = 9;

    
    public final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kTrustMasterPort = 1;
        // d-pad positions
        public static final int kClimbArmUp = 0;
        public static final int kClimbArmExtend = 90;
        public static final int kWinchOnToggle = 180;
        public static final int kClimbArmDownToggle = 270;
		
    }
    public final class AutoConstants {
        public static final double kTimeOut = 1.0;
		public static final double kPower = -0.8;
    }
    public final class DriveConstants{
        public static final int kLeftMotor00CanBusID = 10;
        public static final int kLeftMotor01CanBusID = 11;
        public static final int kRightMotor02CanBusID = 12;
        public static final int kRightMotor03CanBusID = 13;
        public static final double kSlowMaxSpeed = 0.5;
		public static final double kMaxSpeed = 0.9;
    }
    public final class IntakeConstants{
        public static final int kLowerMotorPWM = 2;
        public static final int kUpperMotorPWM = 1;
        public static final int kIntakeMotorPWM = 4;
        public static final double kIntakePower = 0.4; // 0.3
        public static final double kConveyorPulsePower = -0.4;
		public static final double kConveyorFullPower = -0.6; //-0.6
		public static final double kConveyorFiveBallEmpty = 2.0;
        public static final double kConveyorOneBallPulse = 0.13;
		public static final double kUpperLowerRatio = 1.5; // original 1.0, changed to put top spin on ball
        public static final double kLowerTime = 0.15; //0.13
        public static final double kUpperTime = 0.15;
        /*
        public static final double kUpperPower = -0.6;
        public static final double kLowerPower = -0.6; // -0.4 upped for arm passage
		*/
    }
    public final class ArmConstants{
        public static final int kArmMotorPWM = 0;
        public static final int kWinchPWM = 3;
        public static final double kArmPower = 0.3;
        // postive, rachet CW; negative, rachet CCW
		public static final double kWinchPower = -1.0;//-0.5
		public static final int kchannelA = 0;
        public static final int kchannelB = 1;
        public static final double kSetPoint = -315.0;
        public static final double kSetTolerance = 0.10;
		public static final double kEncoderP = 0.0016;
		public static final double kEncoderI = 0.0;
        public static final double kEncoderD = 0.0;
        public static final double countsPerRevolution = 7.0;
        public static final double reduction = 71.1;
		public static final int kPinchSolenoidPortA = 2;
		public static final int kPinchSolenoidPortB = 3;
		public static final int kArmSolenoidPortA = 0;
		public static final int kArmSolenoidPortB = 1;
    }
}
