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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
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
        public static final double kMaxSpeedMetersPerSecond = 0.8; // was 0.8
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.80; // was 0.8
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
    }
    // auto trajectory
    public static final class TrajectoryConstants {
        private static final double kWheelDiameterMeter = Units.inchesToMeters(7.25);
        public static final double kWheelDiameter = kWheelDiameterMeter;

    //Path 1 Points:
        public static final List<Translation2d> pathApartAPoints = List.of(
                new Translation2d(Units.inchesToMeters(+30.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+22.0), Units.inchesToMeters(+10.0))
                );

        public static final List<Translation2d> pathApartBPoints = List.of(
                new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+22.0), Units.inchesToMeters(+10.0))
        );
        public static final List<Translation2d> pathApartCPoints = List.of(
                new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+16.0), Units.inchesToMeters(+16.0)),
                new Translation2d(Units.inchesToMeters(+22.0), Units.inchesToMeters(+10.0))
        );


    //Pose Start+End PartA:
        public static final Pose2d pathApartAstartPose =
                new Pose2d(Units.inchesToMeters(+90.0), Units.inchesToMeters(+30.0), new Rotation2d(Units.degreesToRadians(-90)));
        public static final Pose2d pathApartAendPose = 
                new Pose2d(Units.inchesToMeters(+90.0), Units.inchesToMeters(+138.0), new Rotation2d(Units.degreesToRadians(-90)));

    //Pose Start+End PartB:
        public static final Pose2d pathApartBstartPose =
                new Pose2d(Units.inchesToMeters(+96.0), Units.inchesToMeters(+174.0), new Rotation2d(Units.degreesToRadians(-90)));
        public static final Pose2d pathApartBendPose = 
                new Pose2d(Units.inchesToMeters(+90.0), Units.inchesToMeters(+222.0), new Rotation2d(Units.degreesToRadians(-90)));
    
    //Pose Start+End PartC:
    public static final Pose2d pathApartCstartPose =
    new Pose2d(Units.inchesToMeters(+72.0), Units.inchesToMeters(+252.0), new Rotation2d(Units.degreesToRadians(-90)));
    public static final Pose2d pathApartCendPose = 
    new Pose2d(Units.inchesToMeters(+90.0), Units.inchesToMeters(+30.0), new Rotation2d(Units.degreesToRadians(-90)));
    }

    // copied from chief delphi example
    public static final class DriveConstants {
        // Feedforward Analysis: kS
        public static final double ksVolts = 0.0799;
        // Feedforward Analysis: kV
        public static final double kvVoltSecondsPerMeter = 0.129;
        // Feedforward Analysis: kA
        public static final double kaVoltSecondsSquaredPerMeter = 0.018;
        // Optimal Controller Gain for preset "WPILib (2020-)": kP
        public static final double kPDriveVel = 0.804;

        // check this width
        public static final double kTrackwidthMeters = Units.inchesToMeters(19.75);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
        
        public static final int kLeftMotor00CanBusID = 10;
        public static final int kLeftMotor01CanBusID = 11;
        public static final int kRightMotor02CanBusID = 12;
        public static final int kRightMotor03CanBusID = 13;
        public static final double kSlowMaxSpeed = 0.5;
        public static final double kMaxSpeed = 0.9;
      }


    // on / off switches
	public static boolean powerState = false;
	
    // XBox mappings
    /*  kBumperLeft(5)  --> half speed driving
        kBumperRight(6) --> 
        kStickLeft(9)   --> 
        kStickRight(10) --> 
        kA(1)           --> 
        kB(2)           --> 
        kX(3)           --> 
        kY(4)           --> 
        kBack(7)        --> 
        kStart(8)       --> 
    */
    public static int kSlowDown = XboxController.Button.kBumperLeft.value;
       
    public final class OIConstants{
        public static final int kDriverControllerPort = 0;
    }
    public final class AutoConstants {
        public static final double kTimeOut = 1.0;
		public static final double kPower = -0.8;
    }
    
}
