// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * An instance of this command can be used to manually controll the arm for
 * testing. This should not be used in competition.
 */
public class ManualArmCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private final DoubleSupplier m_shoulderSpeedSupplier;
  private final DoubleSupplier m_elbowSpeedSupplier;

  /**
   * @param armSubsystem          the arm to be manually controlled.
   * @param shoulderSpeedSupplier a supplier of doubles, [-1.0, 1.0] for shoulder
   *                              control.
   * @param elbowSpeedSuppliera   supplier of doubles, [-1.0, 1.0] for elbow
   *                              control.
   */
  public ManualArmCommand(
      final ArmSubsystem armSubsystem,
      final DoubleSupplier shoulderSpeedSupplier,
      final DoubleSupplier elbowSpeedSupplier) {
    m_armSubsystem = armSubsystem;
    m_shoulderSpeedSupplier = shoulderSpeedSupplier;
    m_elbowSpeedSupplier = elbowSpeedSupplier;
    addRequirements(m_armSubsystem);
  }

  /**
   * Read the current values of the suppliers and set the arm speeds.
   */
  @Override
  public void execute() {
    double shoulderSpeed = Math.pow(m_shoulderSpeedSupplier.getAsDouble(), 3.0);
    shoulderSpeed = MathUtil.clamp(shoulderSpeed, -0.25, 0.25);
    double elbowSpeed = Math.pow(m_elbowSpeedSupplier.getAsDouble(), 3.0);
    elbowSpeed = MathUtil.clamp(elbowSpeed, -0.25, 0.25);
    m_armSubsystem.setArmSpeed(shoulderSpeed, elbowSpeed);
  }

  /**
   * Make sure the motors are off when ending.
   */
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setArmSpeed(0.0, 0.0);
    m_armSubsystem.setTargetsToCurrents();
  }
}
