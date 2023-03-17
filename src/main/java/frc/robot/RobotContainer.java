// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.MoveConeHighCommand;
import frc.robot.commands.MoveConeMiddleCommand;
import frc.robot.commands.ScoreAlign;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreConeHighCommand;
import frc.robot.commands.ScoreConeMiddleCommand;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.commands.ZeroElbowCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.LeaveCommunity;
import frc.robot.commands.autonomous.ScoreCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
        modifyAxis(-m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getRightX())
            * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        true),
        m_driveSubsystem));

    m_armSubsystem.setDefaultCommand(
        new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem));

    m_autoChooser.setDefaultOption("Do Nothing", () -> new WaitCommand(1.0));
    m_autoChooser.addOption("LeaveCommRight",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, 0.2));
    m_autoChooser.addOption("LeaveCommLeft",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, -0.2));
    m_autoChooser.addOption("ScoreCubeBalance",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH).andThen(
                new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5.0))
            .andThen(
                new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    m_autoChooser.addOption("ScoreCubeLeaveCommBalance", () -> new ScoreCube(m_driveSubsystem,
        m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH).andThen(
          new DriveUntilCommand(m_driveSubsystem, -1.5, 0, () -> false).withTimeout(3.0)
        ).andThen(new WaitCommand(1.0)).andThen(new BalanceAuto(m_driveSubsystem, 2, 1).withTimeout(0.5)).andThen(
          new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))
        ));
    m_autoChooser.addOption("ScoreCubeStop",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH));
    m_autoChooser.addOption("ScoreCubeLeaveCommRight",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(
                new DriveUntilCommand(m_driveSubsystem, -1, 0.1, () -> false).withTimeout(5)));
    m_autoChooser.addOption("ScoreCubeLeaveCommLeft",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(
                new DriveUntilCommand(m_driveSubsystem, -1, -0.1, () -> false).withTimeout(5)));
    SmartDashboard.putData("Auto chooser", m_autoChooser);

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
        .whileTrue(m_visionSubsystem.centerAprilTagCommand(-Units.inchesToMeters(22),
            Units.inchesToMeters(9)));
    m_primaryController.y()
        .whileTrue(m_visionSubsystem.centerAprilTagCommand(0.0, Units.inchesToMeters(9)));
    m_primaryController.a()
        .whileTrue(m_visionSubsystem.centerAprilTagCommand(0.0, Units.inchesToMeters(9)));
    m_primaryController.x()
        .whileTrue(m_visionSubsystem.centerAprilTagCommand(Units.inchesToMeters(22),
            Units.inchesToMeters(9)));
    //m_primaryController.start().whileTrue(new ScoreAlign(m_driveSubsystem));// .andThen(new
                                                                            // DriveLateral(m_driveSubsystem,
                                                                            // m_visionSubsystem.getLateralDistance(0),
                                                                            // 0.5)));

    // Substation grabs
    m_primaryController.back()
        .whileTrue(m_visionSubsystem
            .centerAprilTagCommand(-Units.inchesToMeters(29.565),
                Units.inchesToMeters(30))
            .andThen(new DriveDistance(m_driveSubsystem, Units.inchesToMeters(26.5),
                1.5)));
    m_primaryController.start()
        .whileTrue(m_visionSubsystem
            .centerAprilTagCommand(Units.inchesToMeters(29.565),
                Units.inchesToMeters(30))
            .andThen(new DriveDistance(m_driveSubsystem, Units.inchesToMeters(26.5),
                1.5)));

    // Driver nudges
    m_primaryController.povUp()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.4, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povDown()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(-0.4, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povLeft()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.4, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povRight()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, -0.4, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.rightTrigger()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.0, 1.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.leftTrigger()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.0, -1.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));

    // Secondary Controller Bindings

    // Zero button for operator

    m_secondaryController.leftStick().and(m_secondaryController.rightStick())
        .onTrue(new ZeroArmCommand(m_armSubsystem));

    // Arm positions
    m_secondaryController.a()
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW)));
    /*
     * m_secondaryController.x()
     * .onTrue(new InstantCommand(
     * () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
     */
    m_secondaryController.axisGreaterThan(Axis.kLeftY.value, 0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_APPROACH)));
    m_secondaryController.leftBumper().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_HALFWAY)));
    m_secondaryController.leftTrigger().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_FULLWAY)));
    m_secondaryController.rightTrigger().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SCORE_LOW))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_armSubsystem
                .setKnownArmPlacement(
                    KnownArmPlacement.FLOOR_GRAB))));

    // Arm nudges
    m_secondaryController.povLeft().onTrue(new InstantCommand(m_armSubsystem::nudgeShoulderBackward));
    m_secondaryController.povRight().onTrue(new InstantCommand(m_armSubsystem::nudgeShoulderForward));
    m_secondaryController.povUp().onTrue(new InstantCommand(m_armSubsystem::nudgeElbowUp));
    m_secondaryController.povDown().onTrue(new InstantCommand(m_armSubsystem::nudgeElbowDown));

    // Game piece indication
    m_secondaryController.start()
        .onTrue(new InstantCommand(
            () -> GameState.getInstance().setGamePieceDesired(GamePiece.CONE)));
    m_secondaryController.back()
        .onTrue(new InstantCommand(
            () -> GameState.getInstance().setGamePieceDesired(GamePiece.CUBE)));

    // Score on driver/operator right bumper press
    Trigger rightBumper = new Trigger(
        (m_primaryController.rightBumper().or(m_secondaryController.rightBumper())));

    // Cube scoring
    Trigger cubeTrigger = new Trigger(
        () -> GameState.getInstance().getGamePieceDesired() == GamePiece.CUBE);
    cubeTrigger.and(m_secondaryController.b())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE)));
    cubeTrigger.and(m_secondaryController.y())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(
                () -> m_armSubsystem.setKnownArmPlacement(
                    KnownArmPlacement.SCORE_CUBE_HIGH))));
    cubeTrigger.and(rightBumper)
        .onTrue(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5));

    // Cone scoring
    Trigger coneTrigger = new Trigger(
        () -> GameState.getInstance().getGamePieceDesired() == GamePiece.CONE);
    coneTrigger.and(m_secondaryController.b())
        .onTrue(new MoveConeMiddleCommand(m_armSubsystem, m_intakeSubsystem).withTimeout(1.5));
    coneTrigger.and(m_secondaryController.y())
        .onTrue(new MoveConeHighCommand(m_armSubsystem, m_intakeSubsystem).withTimeout(1.5));
    Trigger scoreLow = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_LOW);
    coneTrigger.and(scoreLow).and(rightBumper)
        .onTrue(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5));
    Trigger scoreMiddleCone = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CONE_MIDDLE_LOWER);
    coneTrigger.and(scoreMiddleCone).and(rightBumper)
        .onTrue(new ScoreConeMiddleCommand(m_intakeSubsystem).withTimeout(0.5));
    Trigger scoreConeHigh = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CONE_HIGH);
    coneTrigger.and(scoreConeHigh).and(rightBumper)
        .onTrue(new ScoreConeHighCommand(m_intakeSubsystem).withTimeout(0.5));
    /*
     * coneTrigger.and(scoreConeHigh).and(m_secondaryController.x())
     * .onTrue(new InstantCommand( () ->
     * m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
     * .alongWith(new WaitCommand(1))
     * .andThen(new InstantCommand( () ->
     * m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))));
     */

    Trigger scoreCubeHigh = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CUBE_HIGH);

    m_secondaryController.x().onTrue(new ConditionalCommand(
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
            .alongWith(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))),
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)),
        () -> (coneTrigger.and(scoreConeHigh)).or(cubeTrigger.and(scoreCubeHigh)).getAsBoolean()));

    Trigger FloorGrab = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.FLOOR_GRAB);
    FloorGrab.and(cubeTrigger).and(m_secondaryController.x())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_armSubsystem
                .setKnownArmPlacement(KnownArmPlacement.STOWED))));

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

    // Always run intake at hold when not intaking or scoring.
    m_intakeSubsystem.setDefaultCommand(new RunCommand(m_intakeSubsystem::hold, m_intakeSubsystem));
    // Run intake when arm in substation position
    Trigger substationHalfway = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY);
    Trigger substationFullway = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY);
    substationHalfway.or(substationFullway)
        .whileTrue(new IntakeCommand(m_intakeSubsystem).finallyDo(interrupted -> {
          if (!interrupted)
            m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_APPROACH);
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var commandSupplier = m_autoChooser.getSelected();
    if (commandSupplier != null) {
      return commandSupplier.get()
          .alongWith(new RunCommand(() -> m_armSubsystem.proceedToArmPosition(),
              m_armSubsystem))
          .alongWith(new InstantCommand(() -> m_intakeSubsystem.hold()))
          .beforeStarting(new ZeroArmCommand(m_armSubsystem));
    }
    return null;
  }

  public void teleopInit() {
    new ZeroArmCommand(m_armSubsystem).schedule();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
