// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveConeHighCommand;
import frc.robot.commands.MoveConeMiddleCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.commands.autonomous.ConeHighBalance;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.ExitChargeStation;
import frc.robot.commands.autonomous.FlipAuto;
import frc.robot.commands.autonomous.LeaveCommunity;
import frc.robot.commands.autonomous.LowLink;
import frc.robot.commands.autonomous.LowLinkRIGHT;
import frc.robot.commands.autonomous.ScoreCube;
import frc.robot.commands.autonomous.TestOnly;
import frc.robot.commands.autonomous.TwoHighBUMP;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private boolean isAuto;
  public boolean isRedAlliance = (DriverStation.getAlliance() == Alliance.Red);

  private final CommandXboxController m_primaryController = new CommandXboxController(0);
  private final CommandXboxController m_secondaryController = new CommandXboxController(1);
  private final CommandXboxController m_tertiaryController = new CommandXboxController(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
        modifyAxis(m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getRightX())
            * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        true),
        m_driveSubsystem));

    m_armSubsystem.setDefaultCommand(
        new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem));

    m_autoChooser.setDefaultOption("Do Nothing", () -> new DriveUntilCommand(m_driveSubsystem, 0.0, 0.0, () -> true));
    m_autoChooser.addOption("Score1CubeHigh",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH));
    m_autoChooser.addOption("LeaveCommRight",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, 0.2));
    m_autoChooser.addOption("LeaveCommLeft",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, -0.2));
    m_autoChooser.addOption("Score1CubeBalance",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5.0))
            .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    m_autoChooser.addOption("ScoreHighCubeLeaveCommBalance", () -> new ScoreCube(m_driveSubsystem,
        m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
        .andThen(new BalanceAuto(m_driveSubsystem, -3, -1))
        .andThen(new ExitChargeStation(m_driveSubsystem))
        .andThen(new DriveUntilCommand(m_driveSubsystem, -1.0, 0, () -> false).withTimeout(0.5))
        // .andThen(new FlipAuto(m_driveSubsystem).withTimeout(2.0))
        .andThen(new BalanceAuto(m_driveSubsystem, 2, 1))
        .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    m_autoChooser.addOption("ScoreLowCubeLeaveCommBalance",
        () -> new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))
            .andThen(new WaitCommand(0.1))
            .andThen(new ScoreCommand(m_intakeSubsystem).withTimeout(.5))
            .andThen(new BalanceAuto(m_driveSubsystem, -3, -1))
            .andThen(new ExitChargeStation(m_driveSubsystem))
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1.0, 0, () -> false).withTimeout(0.5))
            // .andThen(new FlipAuto(m_driveSubsystem).withTimeout(2.0))
            .andThen(new BalanceAuto(m_driveSubsystem, -2, -1))
            .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    /*
     * .andThen(new BalanceAuto(m_driveSubsystem, -2.0, -1.0).withTimeout(4.0))
     * .andThen(new ExitChargeStation(m_driveSubsystem).withTimeout(4.0)
     * .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0, () ->
     * false).withTimeout(0.75))
     * .andThen(new WaitCommand(0.75))
     * .andThen(new BalanceAuto(m_driveSubsystem, 1.5, 1).withTimeout(5.0))
     * .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01,
     * false))));
     */
    m_autoChooser.addOption("Score1CubeLowLeaveCommRight",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0.1, () -> false).withTimeout(5)));

    m_autoChooser.addOption("Score1CubeLowLeaveCommLeft",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, -0.1, () -> false).withTimeout(5)));

    m_autoChooser.addOption("Score1CubeLowBalance",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
            .andThen(new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5.0))
            .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));

    m_autoChooser.addOption("Score1CubeHighStop",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH));

    m_autoChooser.addOption("Score1CubeHighLeaveCommRight",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0.1, () -> false).withTimeout(5)));

    m_autoChooser.addOption("Score1CubeHighLeaveCommLeft",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, -0.1, () -> false).withTimeout(5)));

    m_autoChooser.addOption("Score3CubeLowLeft",
        () -> new LowLink(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, "BottomLink", 3, 3, isRedAlliance)
            .getCommandAndStop());

    m_autoChooser.addOption("Score2CubeLowBalanceRight",
        () -> new LowLinkRIGHT(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem,
            "ScoreCubeGrabScoreCubeGrabBalance", 2, 1, isRedAlliance).getCommandAndStop()
            .andThen(new BalanceAuto(m_driveSubsystem, 2, 1)));

    m_autoChooser.addOption("Score1ConeHighBalance",
        () -> new MoveConeHighCommand(m_armSubsystem)
            .andThen(new ScoreCommand(m_intakeSubsystem))
            .andThen(new ConeHighBalance(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, "ConeHighBalance", 3, 2,
                isRedAlliance).getCommandAndStop()
                .andThen(new BalanceAuto(m_driveSubsystem, 2, 1))));

    m_autoChooser.addOption("Score2ConeHighRight",
        () -> (new InstantCommand(() -> GameState.getInstance().setGamePieceDesired(GamePiece.CONE)))
            .andThen(
                new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
                    .andThen(new WaitCommand(1.5)))
            .andThen(
                new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CONE_HIGH_PRE)))
            .andThen(new WaitCommand(1.5))
            .andThen(
                new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CONE_HIGH_WRIST)))
            .andThen(new WaitCommand(1.5))
            .andThen(new ScheduleCommand(new ScoreCommand(m_intakeSubsystem).withTimeout(0.3)))
            .andThen(
                new TwoHighBUMP(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, "TwoHighAuto", 1, 1, isRedAlliance)
                    .getCommandAndStop()));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * 
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

    Trigger scoringPosTrigger = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CONE_MIDDLE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_MIDDLE_CUBE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_MIDDLE_CUBE);

    Trigger highCone = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CONE_HIGH);

    // Scoring
    // m_primaryController.x().onTrue(new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SINGLE_SUBSTATION)));
    m_primaryController.rightBumper().whileTrue(new IntakeCommand(m_intakeSubsystem));
    m_primaryController.leftBumper().whileTrue(new ScoreCommand(m_intakeSubsystem));
    // Try onTrue for command actuation, might be interesting
    // Substation grabs
    // Driver nudges
    m_primaryController.povUp()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(-1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povDown()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povLeft()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, -0.8, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povRight()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.8, 0.0, true),
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
    m_secondaryController.leftStick().and(m_secondaryController.rightStick())
        .onTrue(new InstantCommand(() -> m_armSubsystem.seedRelativeEncoders(), m_armSubsystem));
    // scoringPosTrigger.and(m_secondaryController.x().or(m_secondaryController.axisLessThan(Axis.kLeftY.value,
    // -0.5)))
    // .onTrue(new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
    // .andThen(new WaitCommand(5.0))
    // .andThen(new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))));

    // Arm nudges
    m_secondaryController.povLeft().onTrue(new InstantCommand(m_armSubsystem::nudgeShoulderBackward));
    m_secondaryController.povRight().onTrue(new InstantCommand(m_armSubsystem::nudgeShoulderForward));
    m_secondaryController.povUp().onTrue(new InstantCommand(m_armSubsystem::nudgeElbowUp));
    m_secondaryController.povDown().onTrue(new InstantCommand(m_armSubsystem::nudgeElbowDown));
    m_secondaryController.rightTrigger().onTrue(new InstantCommand(m_armSubsystem::nudgeWristDown));
    m_secondaryController.rightBumper().onTrue(new InstantCommand(m_armSubsystem::nudgeWristUp));

    // Game piece indication
    m_secondaryController.start()
        .onTrue(new InstantCommand(
            () -> GameState.getInstance().setGamePieceDesired(GamePiece.CONE)));
    m_secondaryController.back()
        .onTrue(new InstantCommand(
            () -> GameState.getInstance().setGamePieceDesired(GamePiece.CUBE)));

    // Substation arm positions
    // (scoringPosTrigger.negate()).and(highCone.negate()).and(m_secondaryController.axisGreaterThan(Axis.kLeftY.value,
    // 0.5)
    // .or(m_secondaryController.x()))
    // .onTrue(new InstantCommand(
    // () -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)
        .onTrue(new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_APPROACH)));

    Trigger approachTrigger = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_APPROACH);

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
    cubeTrigger.and(m_primaryController.y()).onTrue(
        new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_PRE))
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CUBE))));

    // Cone positions
    Trigger coneTrigger = new Trigger(
        () -> GameState.getInstance().getGamePieceDesired() == GamePiece.CONE);

    Trigger stowedTrigger = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.STOWED);

    coneTrigger.and(m_secondaryController.a()).onTrue(new InstantCommand(() -> m_armSubsystem
        .setKnownArmPlacement(KnownArmPlacement.SCORE_LOW_CONE)));
    coneTrigger.and(m_secondaryController.b())
        .onTrue(new MoveConeMiddleCommand(m_armSubsystem).withTimeout(2.5));
    coneTrigger.and(m_secondaryController.y()).and(stowedTrigger.negate())
        .onTrue(new MoveConeHighCommand(m_armSubsystem));
    coneTrigger.and(m_secondaryController.leftBumper()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CONE)));
    // coneTrigger.and(m_secondaryController.leftTrigger()).onTrue(
    // new InstantCommand(
    // () -> m_armSubsystem.setKnownArmPlacement(
    // KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CONE)));
    (m_secondaryController.axisGreaterThan(Axis.kLeftY.value, 0.5)).or(m_secondaryController.x())
        .onTrue(new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
            .andThen(new WaitCommand(1.0))
            .andThen(new InstantCommand(
                () -> m_armSubsystem.setKnownArmPlacement(
                    KnownArmPlacement.STOWED))));
    // .onTrue((new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
    // .andThen(new WaitCommand(3))
    // .andThen(new InstantCommand(() ->
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)))));

    // cubeTrigger.and(m_secondaryController.y())
    // .onTrue(new InstantCommand(() -> m_armSubsystem
    // .setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
    // .andThen(new WaitCommand(1.0))
    // .andThen(new InstantCommand(
    // () -> m_armSubsystem.setKnownArmPlacement(
    // KnownArmPlacement.SCORE_CUBE_HIGH))));

    coneTrigger.and(stowedTrigger).and(m_secondaryController.y())
        .onTrue(new MoveConeHighCommand(m_armSubsystem));

    coneTrigger.and(m_primaryController.y()).onTrue(
        new InstantCommand(() -> m_armSubsystem
            .setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_PRE))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CONE))));

    // Grabby thing control
    m_intakeSubsystem.setDefaultCommand(
        new RunCommand(() -> m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_HOLD_SPEED), m_intakeSubsystem));

    Trigger substationGrabTrigger = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CUBE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_HALFWAY_CONE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CUBE
            || m_armSubsystem.getArmPlacement() == KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CONE);

    scoringPosTrigger.and(m_secondaryController.x().or(m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)));

    // substationGrabTrigger.onTrue(new IntakeCommand(m_intakeSubsystem)
    // .finallyDo(interrupted -> {
    // if (!interrupted) {
    // m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH);
    // }
    // }));

    Trigger floorGrab = new Trigger(
        () -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.FLOOR_GRAB_CUBE);
    (floorGrab.and(() -> isAuto)).whileTrue(new IntakeCommand(m_intakeSubsystem));
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
    Trigger gamePieceHeld = new Trigger(() -> GameState.getInstance().isGamePieceHeld());

    coneTrigger.and(m_secondaryController.leftTrigger()).onTrue(
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SINGLE_SUBSTATION_CONE)));
    cubeTrigger.and(m_secondaryController.leftTrigger()).onTrue(
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SINGLE_SUBSTATION_CUBE)));

    // floorGrab.and(gamePieceHeld.negate()).whileTrue(new
    // IntakeCommand(m_intakeSubsystem));

    // Tertiary Controller Bindings
  }

  public void changeAutoTrigger(boolean inAuto) {
    this.isAuto = inAuto;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var autoCommandSupplier = m_autoChooser.getSelected();
    if (autoCommandSupplier != null) {
      return autoCommandSupplier.get()
          .alongWith(new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem))
          .alongWith(new InstantCommand(() -> m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_HOLD_SPEED)))
          .beforeStarting(
              (new RunCommand(() -> m_armSubsystem.setManualWristSpeed(-0.1), m_armSubsystem)).withTimeout(0.25));
      // .beforeStarting(new ZeroArmCommand(m_armSubsystem));
    }
    return null;
  }

  public void teleopInit() {
    m_armSubsystem.seedRelativeEncoders();
    m_armSubsystem.setTargetsToCurrents();
    // new ZeroArmCommand(m_armSubsystem).schedule();
  }

  public void autoInit() {
    m_armSubsystem.seedRelativeEncoders();
    m_armSubsystem.setTargetsToCurrents();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void endAutoScore() {
    new ScheduleCommand(new ScoreCommand(m_intakeSubsystem)).schedule();
  }
}
