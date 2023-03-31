// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveLateral;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveConeHighCommand;
import frc.robot.commands.MoveConeMiddleCommand;
import frc.robot.commands.ScoreAlign;
import frc.robot.commands.ScoreAutoCube;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.commands.ZeroElbowCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.commands.Zeroing;
import frc.robot.commands.autonomous.AutoTrajectory;
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.commands.autonomous.DriveUntilCommand;
import frc.robot.commands.autonomous.ExitChargeStation;
import frc.robot.commands.autonomous.FlipAuto;
import frc.robot.commands.autonomous.LeaveCommunity;
import frc.robot.commands.autonomous.LowLink;
import frc.robot.commands.autonomous.LowLinkRIGHT;
import frc.robot.commands.autonomous.ScoreCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

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

    m_autoChooser.addOption("LeaveCommRight",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, 0.2));
    m_autoChooser.addOption("LeaveCommLeft",
        () -> new LeaveCommunity(m_driveSubsystem, m_armSubsystem, -0.2));
    m_autoChooser.addOption("ScoreCubeBalance",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5.0))
            .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    m_autoChooser.addOption("ScoreCubeLeaveCommBalance", () -> new ScoreCube(m_driveSubsystem,
        m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH) 
        .andThen(new DriveUntilCommand(m_driveSubsystem, -1.0, 0, () -> false).withTimeout(3.0))
        .andThen(new FlipAuto(m_driveSubsystem).withTimeout(2.0))
        .andThen(new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5))
        .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
          /*.andThen(new BalanceAuto(m_driveSubsystem, -2.0, -1.0).withTimeout(4.0))
          .andThen(new ExitChargeStation(m_driveSubsystem).withTimeout(4.0)
          .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0, () -> false).withTimeout(0.75))
          .andThen(new WaitCommand(0.75))
          .andThen(new BalanceAuto(m_driveSubsystem, 1.5, 1).withTimeout(5.0))
          .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));*/
    m_autoChooser.addOption("ScoreCubeLowLeaveCommRight", () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
          .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0.1, () -> false).withTimeout(5)));
    m_autoChooser.addOption("ScoreCubeLowLeaveCommLeft", () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
          .andThen(new DriveUntilCommand(m_driveSubsystem, -1, -0.1, () -> false).withTimeout(5)));
    m_autoChooser.addOption("ScoreCubeLowBalance", () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_LOW_CUBE)
          .andThen(new BalanceAuto(m_driveSubsystem, -2, -1).withTimeout(5.0))
          .andThen(new InstantCommand(() -> m_driveSubsystem.drive(0, 0, 0.01, false))));
    m_autoChooser.addOption("ScoreCubeStop",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem,
            m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH));
    m_autoChooser.addOption("ScoreCubeLeaveCommRight",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, 0.1, () -> false).withTimeout(5)));
    m_autoChooser.addOption("ScoreCubeLeaveCommLeft",
        () -> new ScoreCube(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, KnownArmPlacement.SCORE_CUBE_HIGH)
            .andThen(new DriveUntilCommand(m_driveSubsystem, -1, -0.1, () -> false).withTimeout(5)));
    m_autoChooser.addOption("TestPath",
         () -> new LowLink(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, "BottomLink", 3, 3).getCommandAndStop());

    m_autoChooser.addOption("ScoreCubeGrabScoreCubeGrabBalance", 
        () -> new LowLinkRIGHT(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem, "ScoreCubeGrabScoreCubeGrabBalance", 2, 1).getCommandAndStop()
            .andThen(new BalanceAuto(m_driveSubsystem, 2, 1)));

    SmartDashboard.putData("Auto chooser", m_autoChooser);

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

    // Scoring
    m_primaryController.b()
        .whileTrue(new ScoreAlign(m_driveSubsystem)
            .andThen(new DriveLateral(m_driveSubsystem, m_visionSubsystem, -Units.inchesToMeters(18), 0.05)));
    m_primaryController.y().onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SCORE_LOW_CUBE))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_armSubsystem
                .setKnownArmPlacement(
                    KnownArmPlacement.FLOOR_GRAB))));
    m_primaryController.a()
        .whileTrue(new ScoreAlign(m_driveSubsystem)
            .andThen(new DriveLateral(m_driveSubsystem, m_visionSubsystem, Units.inchesToMeters(0), 0.05)));
    m_primaryController.x()
        .whileTrue(new ScoreAlign(m_driveSubsystem)
            .andThen(new DriveLateral(m_driveSubsystem, m_visionSubsystem, Units.inchesToMeters(18), 0.05)));
    // Try onTrue for command actuation, might be interesting
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
        .onTrue(new Zeroing(m_armSubsystem));

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
    (m_secondaryController.axisGreaterThan(Axis.kLeftY.value, 0.5)
        .or(m_secondaryController.x()))
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
    coneTrigger.and(m_secondaryController.leftTrigger()).onTrue(
        new InstantCommand(
            () -> m_armSubsystem.setKnownArmPlacement(
                KnownArmPlacement.SUBSTATION_GRAB_FULLWAY_CONE)));
    Trigger highCone = new Trigger(() -> m_armSubsystem.getArmPlacement() == KnownArmPlacement.SCORE_CONE_HIGH);
        coneTrigger.and(highCone).and((m_secondaryController.axisLessThan(Axis.kLeftY.value, -0.5)).or(m_secondaryController.x()))
          .onTrue(new InstantCommand(() ->
          m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CONE_HIGH_RETURN))
          .alongWith(new WaitCommand(2))
          .andThen(new InstantCommand(() ->
          m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))));

        coneTrigger.and(stowedTrigger).and(m_secondaryController.y())
          .onTrue(new MoveConeHighCommand(m_armSubsystem));
        

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

    m_secondaryController.rightBumper().whileTrue(new ScoreCommand(m_intakeSubsystem));
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
