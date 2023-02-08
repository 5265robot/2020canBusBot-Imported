package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.rAutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  // drn -- robot's subsystems and commands are defined here...

  // drn -- drive & intake & arm subsystem declarations
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // drn --- declaring an instance of the XBox controller
  private final XboxController m_xboxController = new XboxController(OIConstants.kDriverControllerPort);
  
  // drn -- A chooser for autonomous commands
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // drn -- while driving diagnostics
  public final PowerDistributionPanel pdp = new PowerDistributionPanel();

  // drn -- simple autonomous driving
  // use ()-> to specify the start and end commands
  private final Command m_simpleDriveForward = new RunCommand(() -> m_robotDrive.arcadeDrive(AutoConstants.kPower, 0.0),
      m_robotDrive).withTimeout(AutoConstants.kTimeOut);

  private final Command m_simpleDriveReverse = new StartEndCommand(
      () -> m_robotDrive.arcadeDrive(-AutoConstants.kPower, 0.0), () -> m_robotDrive.arcadeDrive(0.0, 0.0),
      m_robotDrive).withTimeout(AutoConstants.kTimeOut);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // drive command to split-stick arcade drive
    // split stick is left and right sticks on the XBox
    m_robotDrive
        .setDefaultCommand(new RunCommand(() -> m_robotDrive.arcadeDrive(m_xboxController.getY(GenericHID.Hand.kLeft),
            m_xboxController.getX(GenericHID.Hand.kRight)), m_robotDrive));

    // drn -- sets up the driver's station to have options for autonomous
    m_chooser.addOption("Forward Auto", m_simpleDriveForward);
    m_chooser.addOption("Reverse Auto", m_simpleDriveReverse);
    m_chooser.setDefaultOption("ramsete path A", generatePathARamseteCommand());

    Shuffleboard.getTab("Auto").add(m_chooser);

  } // end RobotContainer initialization methods

    // ramsete command
    private Command generatePathARamseteCommand() {
      var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                         DriveConstants.kvVoltSecondsPerMeter, 
                                         DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics,
              10);
    TrajectoryConfig config =
      new TrajectoryConfig(rAutoConstants.kMaxSpeedMetersPerSecond, 
                          rAutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics)
          .addConstraint(autoVoltageConstraint);
    RamseteController m_rController = new RamseteController(rAutoConstants.kRamseteB, rAutoConstants.kRamseteZeta);
    SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.kvVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter);
    Trajectory pathApartATrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      TrajectoryConstants.pathApartAstartPose,
      // which path and part
      TrajectoryConstants.pathApartAPoints,
      TrajectoryConstants.pathApartAendPose, 
      config);
    Trajectory pathApartBTrajectory = TrajectoryGenerator.generateTrajectory(TrajectoryConstants.pathApartBstartPose, TrajectoryConstants.pathApartBPoints, TrajectoryConstants.pathApartBendPose, config);
    Trajectory pathApartCTrajectory = TrajectoryGenerator.generateTrajectory(TrajectoryConstants.pathApartCstartPose, TrajectoryConstants.pathApartCPoints, TrajectoryConstants.pathApartCendPose, config);
    
    RamseteCommand pathApartAramseteCommand = new RamseteCommand(
        pathApartATrajectory,
        m_robotDrive::getPose, m_rController, m_Feedforward, DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand pathApartBramseteCommand = new RamseteCommand(
        pathApartBTrajectory,
        m_robotDrive::getPose, m_rController, m_Feedforward, DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), m_robotDrive::tankDriveVolts, m_robotDrive);
    RamseteCommand pathApartCramseteCommand = new RamseteCommand(
        pathApartCTrajectory,
        m_robotDrive::getPose, m_rController, m_Feedforward, DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), m_robotDrive::tankDriveVolts, m_robotDrive);

    m_robotDrive.resetOdometry(pathApartATrajectory.getInitialPose());

    return new InstantCommand(
      () -> m_robotDrive.resetOdometry(pathApartATrajectory.getInitialPose()),m_robotDrive)
      .andThen(pathApartAramseteCommand).andThen(pathApartBramseteCommand).andThen(pathApartCramseteCommand)
      .andThen(new InstantCommand(() -> m_robotDrive.tankDriveVolts(0, 0),m_robotDrive));
    }


  private void configureButtonBindings() {
    // speed
    final JoystickButton slowDown = new JoystickButton(m_xboxController, Constants.kSlowDown);
    slowDown.whenPressed(() -> m_robotDrive.halfPower());
  } // end configureButtonBindins

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  } // end getAutonomous

} // end RobotContainer
