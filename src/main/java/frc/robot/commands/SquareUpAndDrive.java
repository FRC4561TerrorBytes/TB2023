// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SquareUpAndDrive extends CommandBase {
  /**
   * Not actually using a trapezoid profile. Using the constraint types is,
   * however, a convenient way to define our limits.
   */
  private static final TrapezoidProfile.Constraints MAX_VEL_ACCEL = new TrapezoidProfile.Constraints(2.0, 1.0);
  private static final double ALLOWED_ACCEL_PER_20MS_TIMESLICE = 0.02 * MAX_VEL_ACCEL.maxAcceleration;
  private static final double MINIMUM_FORWARD_VELOCITY = 0.1;

  final DriveSubsystem m_driveSubsystem;
  final PIDController m_rotationController = new PIDController(0.03, 0, 0);
  final DoubleSupplier m_forwardSupplier;
  double m_lastForwardVelocity = 0.0;
  final DoubleSupplier m_strafeSupplier;

  public SquareUpAndDrive(final DriveSubsystem driveSubsystem,
      final DoubleSupplier forwardSupplier,
      final DoubleSupplier strafeSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_forwardSupplier = forwardSupplier;
    m_strafeSupplier = strafeSupplier;
    addRequirements(m_driveSubsystem);
    m_rotationController.enableContinuousInput(-180.0, 180.0);
    m_rotationController.setTolerance(0.05);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationController.reset();
    final double absAngle = Math.abs(m_driveSubsystem.getPose().getRotation().getDegrees());
    final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    m_rotationController.setSetpoint(closerTo0 ? 0.0 : 180.0);
    m_lastForwardVelocity = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the rotation
    final double rawAngle = m_driveSubsystem.getPose().getRotation().getDegrees();
    double rotationRate = m_rotationController.calculate(rawAngle);
    rotationRate += 1.0 * Math.signum(rotationRate);

    // For now, do not limit the strafe.
    final double strafeVelocity = m_strafeSupplier.getAsDouble();

    // Get the forward but limit the acceleration.
    double forwardVelocity = m_forwardSupplier.getAsDouble();
    if (Math.abs(m_lastForwardVelocity) >= MAX_VEL_ACCEL.maxVelocity) {
      // Already at maximum
      if (Math.abs(forwardVelocity) >= MAX_VEL_ACCEL.maxVelocity) {
        // Request is maximum or more, stay at max.
        forwardVelocity = m_lastForwardVelocity;
      }
    } else {
      // Changing velocity but not at max yet.
      if (m_lastForwardVelocity == 0.0) {
        // Special case just starting.
        if (forwardVelocity != 0.0) {
          // The request is to start moving, first go to the minimum.
          forwardVelocity = MINIMUM_FORWARD_VELOCITY * Math.signum(forwardVelocity);
        }
      } else {
        // Was already moving but may want to change.
        final double requestedDelta = forwardVelocity - m_lastForwardVelocity;
        double limitedDelta = Math.min(ALLOWED_ACCEL_PER_20MS_TIMESLICE, Math.abs(requestedDelta))
            * Math.signum(requestedDelta);
        forwardVelocity = m_lastForwardVelocity + limitedDelta;
        if (Math.abs(forwardVelocity) > MAX_VEL_ACCEL.maxVelocity) {
          // Just calculated going over max velocity.
          forwardVelocity = MAX_VEL_ACCEL.maxVelocity * Math.signum(forwardVelocity);
        } else if (Math.abs(forwardVelocity) < MINIMUM_FORWARD_VELOCITY) {
          // Just slowed to less than minimum, stop going forward.
          forwardVelocity = 0.0;
        }
      }
    }

    m_driveSubsystem.drive(
        forwardVelocity,
        strafeVelocity,
        rotationRate,
        true);

    m_lastForwardVelocity = forwardVelocity;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }
}
