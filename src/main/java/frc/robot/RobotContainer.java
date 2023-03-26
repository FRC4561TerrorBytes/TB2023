// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveConeHighCommand;
import frc.robot.commands.MoveConeMiddleCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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

  private final CommandXboxController m_secondaryController = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_armSubsystem.setDefaultCommand(
        new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem));

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
    // Secondary Controller Bindings

    // Zero button for operator
    m_secondaryController.leftStick().and(m_secondaryController.rightStick())
        .onTrue(new ZeroArmCommand(m_armSubsystem));

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

    // Substation arm positions
    m_secondaryController.axisGreaterThan(Axis.kLeftY.value, 0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_APPROACH)));

    // Cube Positions
    Trigger cubeTrigger = new Trigger(
        () -> GameState.getInstance().getGamePieceDesired() == GamePiece.CUBE);

    cubeTrigger.and(m_secondaryController.a())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW_CUBE)));
    cubeTrigger.and(m_secondaryController.b())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE_CUBE)));
    cubeTrigger.and(m_secondaryController.y())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(
                () -> m_armSubsystem.setKnownArmPlacement(
                    KnownArmPlacement.SCORE_CUBE_HIGH))));
    cubeTrigger.and(m_secondaryController.leftBumper()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CUBE)));
    cubeTrigger.and(m_secondaryController.leftTrigger()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CUBE)));

    // Cone positions
    Trigger coneTrigger = new Trigger(
        () -> GameState.getInstance().getGamePieceDesired() == GamePiece.CONE);

    coneTrigger.and(m_secondaryController.a()).onTrue(new InstantCommand(() -> m_armSubsystem
        .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW_CONE)));
    coneTrigger.and(m_secondaryController.b())
        .onTrue(new MoveConeMiddleCommand(m_armSubsystem).withTimeout(2.5));
    coneTrigger.and(m_secondaryController.y())
        .onTrue(new MoveConeHighCommand(m_armSubsystem).withTimeout(1.5));
    coneTrigger.and(m_secondaryController.leftBumper()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CONE)));
    coneTrigger.and(m_secondaryController.leftTrigger()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CONE)));

    // Grabby thing control
    m_intakeSubsystem.setDefaultCommand(
        new RunCommand(() -> m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_HOLD_SPEED), m_intakeSubsystem));

    Trigger substationGrabTrigger = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CUBE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CONE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CUBE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CONE);

    substationGrabTrigger.onTrue(new IntakeCommand(m_intakeSubsystem)
        .finallyDo(interrupted -> {
          if (!interrupted) {
            m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH);
          }
        }));

    m_secondaryController.rightBumper().onTrue(new ScoreCommand(m_intakeSubsystem));
    // Trigger floorGrab = new Trigger(
    // () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.FLOOR_GRAB);
    // m_secondaryController.x().and(floorGrab.negate()).onTrue(new
    // ConditionalCommand(
    // new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
    // .alongWith(new WaitCommand(1))
    // .andThen(new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))),
    // new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)),
    // () ->
    // (coneTrigger.and(scoreConeHigh)).or(cubeTrigger.and(scoreCubeHigh)).getAsBoolean()));

    // floorGrab.and(m_secondaryController.x())
    // .onTrue(new InstantCommand(() -> m_armSubsystem
    // .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW))
    // .andThen(new WaitCommand(0.5))
    // .andThen(new InstantCommand(() -> m_armSubsystem
    // .setKnownArmPlacement(KnownArmPlacement.STOWED))));
    // Trigger gamePieceHeld = new Trigger(() ->
    // GameState.getInstance().isGamePieceHeld());
    // floorGrab.and(gamePieceHeld.negate()).whileTrue(new
    // IntakeCommand(m_intakeSubsystem));

    // Tertiary Controller Bindings
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

  public void teleopInit() {
    // new ZeroArmCommand(m_armSubsystem).schedule();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
