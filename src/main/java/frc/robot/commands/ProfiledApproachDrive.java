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
  /** Constraints to use if April tag found and we know the distance (faster). */
  private static final TrapezoidProfile.Constraints TAG_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 1.0);
  /** If a tag is found, profile to drive this much past it. */
  private static final double TAG_DRIVE_THROUGH_TARGET_METERS = 0.15;
  /** Constraints to use if no April tag found (slower). */
  private static final double NO_TAG_FOUND_VALUE = -1.0;
  private static final TrapezoidProfile.Constraints NO_TAG_CONSTRAINTS = new TrapezoidProfile.Constraints(1.25, 1.0);
  /** Profile to drive this distance if no tag seen. */
  private static final double NOT_TAG_ASSUMED_DISTANCE_METERS = 5.0;
  /** Minimum drive velocity to avoid starting pause and short drives. */
  private static final double MINIMUM_VELOCITY_METERS_PER_SECOND = 0.1;

  private final DriveSubsystem m_driveSubsystem;
  private final GameState m_gameState;

  /**
   * Creates a new {@link ProfiledApproachDrive} command.
   * 
   * @param driveSubsystem  the {@link DriveSubsystem} to update.
   * @param visionSubsystem the {@link VisionSubsystem} for April tag detection.
   */
  public ProfiledApproachDrive(
      final DriveSubsystem driveSubsystem,
      final VisionSubsystem visionSubsystem) {
    super(
        () -> calculateProfile(visionSubsystem),
        setpointState -> driveForward(setpointState, driveSubsystem),
        // Require the drive, not the vision.
        driveSubsystem);

    m_driveSubsystem = driveSubsystem;
    m_gameState = GameState.getInstance();
  }

  /**
   * Ends when the super implementation returns true or we have a game piece.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public boolean isFinished() {
    return super.isFinished() || m_gameState.isGamePieceHeld();
  }

  /**
   * Calls the super implementation and stops the {@link DriveSubsystem}.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_driveSubsystem.stop();
  }

  /**
   * Uses the {@link VisionSubsystem} to find the distance to a April tag before
   * approach. If no tag is found or it is not valid, {@link #NO_TAG_FOUND_VALUE}
   * is returned.
   * 
   * @param visionSubsystem the {@link VisionSubsystem} to use to find the tag.
   * @return the distance from the front of the bumper to the tag, or
   *         {@link #NO_TAG_FOUND_VALUE} if no valid tag found.
   */
  private static TrapezoidProfile calculateProfile(final VisionSubsystem visionSubsystem) {
    double distance = visionSubsystem.getTargetDistance();
    SmartDashboard.putBoolean("Approach Tag", distance != NO_TAG_FOUND_VALUE);
    TrapezoidProfile.Constraints constraints = TAG_CONSTRAINTS;
    if (distance == NO_TAG_FOUND_VALUE) {
      // No good target... ramming speed!
      distance = NOT_TAG_ASSUMED_DISTANCE_METERS;
      constraints = NO_TAG_CONSTRAINTS;
    } else {
      distance += TAG_DRIVE_THROUGH_TARGET_METERS;
    }
    return new TrapezoidProfile(
        // Limit the max acceleration and velocity
        constraints,
        // Arbitrary long distance. We want the smooth accel.
        new TrapezoidProfile.State(distance, 0));
  }

  /**
   * The drive was factored out to this method for ease of enforcing a minimum
   * velocity. This is intended to remove the hesitation we saw at the start and
   * should help keep us from coming up short on the distance.
   * 
   * @param setpointState  the {@link TrapezoidProfile.State} from the profile
   *                       calculation.
   * @param driveSubsystem the {@link DriveSubsystem} to update.
   */
  private static void driveForward(final TrapezoidProfile.State setpointState, final DriveSubsystem driveSubsystem) {
    double xVelocity = Math.abs(setpointState.velocity);
    if (xVelocity < MINIMUM_VELOCITY_METERS_PER_SECOND) {
      xVelocity = MINIMUM_VELOCITY_METERS_PER_SECOND;
    }
    driveSubsystem.drive(
        xVelocity,
        0, 0, false);
  }
}
