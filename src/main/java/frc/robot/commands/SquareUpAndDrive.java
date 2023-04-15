// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.FieldRelativeSpeeds;

/**
 * This command squares the robot up to the nearest 909 degrees and limits the
 * velocity rate of change (accel and decel) for both the X and Y directions.
 * The goal is to allow the driver to continue to control the robot on approach
 * to a substation or scoring position while keeping the arm steady.
 * 
 * <p>
 * Note that the driver does NOT need to stop before initiating this drive mode.
 * The slew rate limiters are initialized with the current speeds.
 * </p>
 */
public class SquareUpAndDrive extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController m_rotationController = new PIDController(0.03, 0, 0);
  private final DoubleSupplier m_forwardSupplier;
  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1.0);
  private final DoubleSupplier m_strafeSupplier;
  private final SlewRateLimiter m_strafeLimiter = new SlewRateLimiter(1.0);

  public SquareUpAndDrive(final DriveSubsystem driveSubsystem,
      final DoubleSupplier forwardSupplier,
      final DoubleSupplier strafeSupplier) {
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
    final double angle = m_driveSubsystem.getPose().getRotation().getDegrees();
    final double absAngle = Math.abs(angle);
    final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    final boolean closerTo90 = absAngle > 45.0 && absAngle < 135.0;
    if (closerTo90) {
      m_rotationController.setSetpoint(90.0 * Math.signum(angle));
    } else {
      m_rotationController.setSetpoint(closerTo0 ? 0.0 : 180.0);
    }

    // Initialize the SlewRateLimiters with the current speeds for
    // smoothe transition between drive types.
    final FieldRelativeSpeeds fieldRelSpeeds = m_driveSubsystem.getFieldRelativeSpeeds();
    m_forwardLimiter.reset(fieldRelSpeeds.vxMetersPerSecond);
    m_strafeLimiter.reset(fieldRelSpeeds.vyMetersPerSecond);
    // TODO Should we attempt to start squaring rotation at current rate?
    // Probably more trouble than it is worth.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the rotation
    final double rawAngle = m_driveSubsystem.getPose().getRotation().getDegrees();
    double rotationRate = m_rotationController.calculate(rawAngle);
    rotationRate += 1.0 * Math.signum(rotationRate);

    // Get the forward and limit the acceleration.
    final double forwardVelocity = m_forwardLimiter.calculate(m_forwardSupplier.getAsDouble());

    // Get the strafe and limit the acceleration.
    final double strafeVelocity = m_strafeLimiter.calculate(m_strafeSupplier.getAsDouble());

    m_driveSubsystem.drive(
        forwardVelocity,
        strafeVelocity,
        rotationRate,
        true);
  }
}
