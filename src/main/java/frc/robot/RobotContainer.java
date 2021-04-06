package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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

  // drn -- shuffleboard tabs
  private final ShuffleboardTab sbConfig = Shuffleboard.getTab("Config");

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
    m_chooser.setDefaultOption("Forward Auto", m_simpleDriveForward);
    sbConfig.add(m_chooser).withSize(3, 1).withPosition(0, 0);

  } // end RobotContainer initialization methods

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
