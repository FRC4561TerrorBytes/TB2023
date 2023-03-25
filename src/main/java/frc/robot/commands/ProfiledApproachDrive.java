// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GameState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * A {@link CalculatedTrapezoidProfileCommand} that drives forward to the
 * substation with acceleration control to keep the arm stable.
 */
public class ProfiledApproachDrive extends CalculatedTrapezoidProfileCommand {
  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 1.0);
  private final DriveSubsystem m_driveSubsystem;
  private final GameState m_gameState;

  /** Creates a new ProfiledApproachDrive. */
  public ProfiledApproachDrive(
      final DriveSubsystem driveSubsystem,
      final VisionSubsystem visionSubsystem) {
    super(
        () -> calculateProfile(visionSubsystem),
        // We care about the profiled x velocity.
        setpointState -> driveSubsystem.drive(
            Math.abs(setpointState.velocity),
            0, 0, false),
        // Require the drive, not the vision.
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

  private static TrapezoidProfile calculateProfile(final VisionSubsystem visionSubsystem) {
    double distance = visionSubsystem.getTargetDistance();
    SmartDashboard.putBoolean("Approach Tag", distance != -1.0);
    if (distance == -1.0) {
      // No good target... ramming speed!
      distance = 10.0;
    }
    return new TrapezoidProfile(
        // Limit the max acceleration and velocity
        CONSTRAINTS,
        // Arbitrary long distance. We want the smooth accel.
        new TrapezoidProfile.State(distance, 0));
  }
}
