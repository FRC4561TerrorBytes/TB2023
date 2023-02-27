// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ZeroElbowCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_driveSubsystem);
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  private final CommandXboxController m_primaryController = new CommandXboxController(0);
  private final CommandXboxController m_secondaryController = new CommandXboxController(1);
  private final CommandXboxController m_tertiaryController = new CommandXboxController(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
        modifyAxis(-m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        true),
        m_driveSubsystem));

    m_armSubsystem.setDefaultCommand(new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Primary Controller Bindings

    // Scoring
    m_primaryController.b()
        .whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(-Units.inchesToMeters(22)),
            m_driveSubsystem));
    m_primaryController.y()
        .whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(0.0), m_driveSubsystem));
    m_primaryController.x()
        .whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(Units.inchesToMeters(22)),
            m_driveSubsystem));

    // Substation grabs
    m_primaryController.leftBumper()
        .whileTrue(
            new RunCommand(() -> m_visionSubsystem.centerAprilTag(-Units.inchesToMeters(29.565)),
                m_driveSubsystem));
    m_primaryController.rightBumper()
        .whileTrue(
            new RunCommand(() -> m_visionSubsystem.centerAprilTag(Units.inchesToMeters(29.565)),
                m_driveSubsystem));

    // Driver nudges
    m_primaryController.povUp()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.4, 0.0, 0.0, true), m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povDown()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(-0.4, 0.0, 0.0, true), m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povLeft()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, -0.4, 0.0, true), m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povRight()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.4, 0.0, true), m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));

    // Secondary Controller Bindings

    // Arm positions
    m_secondaryController.a()
        .onTrue(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_LOW)));
    m_secondaryController.b()
        .onTrue(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE)));
    m_secondaryController.y().onTrue(
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE))
            .andThen(new WaitCommand(1.0))
            .andThen(new InstantCommand(
                () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH))));
    m_secondaryController.x()
        .onTrue(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)));
    m_secondaryController.axisGreaterThan(Axis.kLeftY.value, 0.5)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_secondaryController.leftBumper().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_GRAB_HALFWAY)));
    m_secondaryController.leftTrigger().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_GRAB_FULLWAY)));

    // Game piece indication
    m_secondaryController.start()
        .onTrue(new InstantCommand(() -> GameState.getInstance().setGamePieceDesired(GamePiece.CONE)));
    m_secondaryController.back()
        .onTrue(new InstantCommand(() -> GameState.getInstance().setGamePieceDesired(GamePiece.CUBE)));

    // Score
    m_secondaryController.rightBumper().onTrue(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5));

    // Tertiary Controller Bindings

    // Switch between manual arm control
    m_tertiaryController.back().onTrue(new ManualArmCommand(
        m_armSubsystem,
        () -> -m_tertiaryController.getLeftY(),
        () -> -m_tertiaryController.getRightY()).until(m_tertiaryController.start()));

    // Re:Zero − Starting Life in Another World
    m_tertiaryController.x().and(m_tertiaryController.y()).onTrue(new ZeroShoulderCommand(m_armSubsystem)
        .alongWith(new RunCommand(() -> m_armSubsystem.setElbowSpeed(0.1)).withTimeout(1.0))
        .andThen(new ZeroElbowCommand(m_armSubsystem)));

    // Miscellaneous Bindings

    // Run intake when arm in substation position
    Trigger substationApproach = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_APPROACH);
    Trigger substationHalfway = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY);
    Trigger substationFullway = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY);
    substationApproach.and(substationHalfway).and(substationFullway)
        .whileTrue(new IntakeCommand(m_intakeSubsystem).finallyDo(interrupted -> {
          if (!interrupted)
            m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH);
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
