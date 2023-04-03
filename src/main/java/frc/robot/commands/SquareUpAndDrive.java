// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SquareUpAndDrive extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PIDController m_rotationController = new PIDController(0.03, 0, 0);
  private final DoubleSupplier m_forwardSupplier;
  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1.0);
  private final DoubleSupplier m_strafeSupplier;

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
    final double absAngle = Math.abs(m_driveSubsystem.getPose().getRotation().getDegrees());
    final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    m_rotationController.setSetpoint(closerTo0 ? 0.0 : 180.0);
    m_forwardLimiter.reset(0.0);
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
    final double forwardVelocity = m_forwardLimiter.calculate(m_forwardSupplier.getAsDouble());

    m_driveSubsystem.drive(
        forwardVelocity,
        strafeVelocity,
        rotationRate,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }
}
