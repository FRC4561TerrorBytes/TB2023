// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that runs a {@link TrapezoidProfile}. Useful for smoothly
 * controlling mechanism motion when the profile varies from run to run.
 */
public class CalculatedTrapezoidProfileCommand extends CommandBase {
  private final Consumer<State> m_output;
  private final Timer m_timer = new Timer();
  private final Supplier<TrapezoidProfile> m_profileSupplier;

  /** A new profile is calculated each time scheduled. */
  private TrapezoidProfile m_profile;

  /**
   * Creates a new CalculatedTrapezoidProfileCommand that will execute a
   * {@link TrapezoidProfile} created during initialize. Output will be piped to
   * the provided consumer function.
   *
   * @param profileSupplier The supplier of the calculate profile.
   * @param output          The consumer for the profile output.
   * @param requirements    The subsystems required by this command.
   */
  public CalculatedTrapezoidProfileCommand(
      Supplier<TrapezoidProfile> profileSupplier, Consumer<State> output, Subsystem... requirements) {
    m_profileSupplier = profileSupplier;
    m_output = output;
    addRequirements(requirements);
  }

  /**
   * Get the new {@link TrapezoidProfile} for this run and restart the profile
   * execution timer.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public void initialize() {
    m_profile = m_profileSupplier.get();
    m_timer.restart();
  }

  /**
   * Get the next output from the profile and send to output.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public void execute() {
    m_output.accept(m_profile.calculate(m_timer.get()));
  }

  /**
   * Stop the profile run timer.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  /**
   * Done when the timer has reached the profile total time.
   * 
   * <p>
   * {@inheritDoc}
   * </p>
   */
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }
}
