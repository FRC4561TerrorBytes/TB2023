// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.AutoTrajectory;
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.subsystems.DriveSubsystem;

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

  private final CommandXboxController m_primaryController = new CommandXboxController(0);
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

    m_autoChooser.addOption("ScoreConeGrabBalanceLEFT", () -> (new AutoTrajectory(m_driveSubsystem, "ScoreGrabBalanceLEFT", 2, 2).getCommandAndStop()
        .andThen(new BalanceAuto(m_driveSubsystem, 2, 1))));

    m_autoChooser.addOption("ScoreConeGrabeBalanceRIGHT", () -> (new AutoTrajectory(m_driveSubsystem, "ScoreConeGrabLeaveRIGHT", 2, 2).getCommandAndStop()
        .andThen(new BalanceAuto(m_driveSubsystem, 2, 1))));

    m_autoChooser.addOption("ScoreCubeGrabScoreCubeGrabBalance", () -> (new AutoTrajectory(m_driveSubsystem, "ScoreCubeGrabScoreCubeGrabBalance", 2, 1).getCommandAndStop()
        .andThen(new BalanceAuto(m_driveSubsystem, 2, 1))));
    m_autoChooser.addOption("TestPath", () -> (new AutoTrajectory(m_driveSubsystem, "S thing ig", 1, 1).getCommandAndStop()));
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

    // Driver nudges
    m_primaryController.povUp()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povDown()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(-1.0, 0.0, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povLeft()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, 0.8, 0.0, true),
            m_driveSubsystem))
        .onFalse(new InstantCommand(() -> m_driveSubsystem.stop()));
    m_primaryController.povRight()
        .whileTrue(new RunCommand(() -> m_driveSubsystem.drive(0.0, -0.8, 0.0, true),
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
    /*
     * coneTrigger.and(scoreConeHigh).and(m_secondaryController.x())
     * .onTrue(new InstantCommand( () ->
     * m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH))
     * .alongWith(new WaitCommand(1))
     * .andThen(new InstantCommand( () ->
     * m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED))));
     */
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
      return commandSupplier.get();
          // .beforeStarting(new ZeroArmCommand(m_armSubsystem));
    }
    return null;
  }

  public void teleopInit() {
    //new ZeroArmCommand(m_armSubsystem).schedule();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
