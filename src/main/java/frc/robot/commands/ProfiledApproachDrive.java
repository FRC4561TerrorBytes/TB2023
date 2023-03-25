// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.GameState;
import frc.robot.subsystems.DriveSubsystem;

public class ProfiledApproachDrive extends TrapezoidProfileCommand {
  final DriveSubsystem m_driveSubsystem;
  final GameState m_gameState;

  /** Creates a new ProfiledApproachDrive. */
  public ProfiledApproachDrive(final DriveSubsystem driveSubsystem) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(
                2.0,
                1.0),
            // Arbitrary long distance. We want the smooth accel.
            new TrapezoidProfile.State(10.0, 0)),
        // We care about the profiled x velocity.
        setpointState -> driveSubsystem.drive(
            Math.abs(setpointState.velocity),
            0, 0, false),
        // Require the drive
        driveSubsystem);

    m_driveSubsystem = driveSubsystem;
    m_gameState = GameState.getInstance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() || m_gameState.isGamePieceHeld();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_driveSubsystem.stop();
  }
}
