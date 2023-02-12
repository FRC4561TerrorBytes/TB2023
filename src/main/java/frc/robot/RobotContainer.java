// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
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
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_driveSubsystem);

  private final CommandXboxController m_primaryController = new CommandXboxController(0);
  // private final CommandXboxController m_secondaryController = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
        modifyAxis(-m_primaryController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(-m_primaryController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(m_primaryController.getRightX())* Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        false),
        m_driveSubsystem));
 
    // m_armSubsystem.setDefaultCommand(
    //     new RunCommand(() -> m_armSubsystem.proceedToArmPosition(), m_armSubsystem));
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
  @SuppressWarnings("unused")
  private void configureBindings() {
    //Primary Controller Vision Bindings
    Trigger primaryButtonA = m_primaryController.a();
    Trigger primaryButtonX = m_primaryController.x();
    Trigger primaryButtonB = m_primaryController.b();
    Trigger primaryLeftBumper = m_primaryController.leftBumper();
    Trigger primaryLeftTrigger = m_primaryController.leftTrigger();
    Trigger primaryRightBumper = m_primaryController.rightBumper();
    Trigger primaryRightTrigger = m_primaryController.rightTrigger();

    primaryButtonX.whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(-Units.inchesToMeters(22)), m_driveSubsystem)); 
    primaryButtonA.whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(0), m_driveSubsystem)); 
    primaryButtonB.whileTrue(new RunCommand(() -> m_visionSubsystem.centerAprilTag(Units.inchesToMeters(22)), m_driveSubsystem)); 

    //driver nudges
    primaryLeftBumper.whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0, 0.4, 0, false), m_driveSubsystem))
                     .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    primaryRightBumper.whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0, -0.4, 0, false), m_driveSubsystem))
                      .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));

    primaryLeftTrigger.whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0, 0, -1, false), m_driveSubsystem))
                      .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    primaryRightTrigger.whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0, 0, 1, false), m_driveSubsystem))
                       .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));

    //Secondary Controller Arm Bindings
    // m_secondaryController.start().onTrue(new InstantCommand(() -> m_armSubsystem.resetPosition()));
    // m_secondaryController.back().toggleOnTrue(new ManualArmCommand(
    //     m_armSubsystem,
    //     () -> m_secondaryController.getLeftY(),
    //     () -> m_secondaryController.getRightY()));
    // m_secondaryController.povUp().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)));
    // m_secondaryController.povDown().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STARTING)));
    // m_secondaryController.leftBumper().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_GRAB_HALFWAY)));
    // m_secondaryController.leftTrigger().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_GRAB_FULLWAY)));
    // m_secondaryController.a().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_LOW)));
    // m_secondaryController.b().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_MIDDLE)));
    // m_secondaryController.y().onTrue(
    //     new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_PREP_INITIAL))
    //         .andThen(new WaitCommand(1.0)) // Cannot find way to call "isOnTarget".
    //         .andThen(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_HIGH)));
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
