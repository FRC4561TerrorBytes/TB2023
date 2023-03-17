// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GameState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private Timer timeout = new Timer();

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    timeout.reset();
  }

  /**
   * Run Intake at speed until motors stall (game piece acquired)
   */
  @Override
  public void execute() {
    m_intakeSubsystem.intake();
    if (m_intakeSubsystem.isStalled()) {
      timeout.start();
    }
  }

  /**
   * Set Game State to Held when IntakeCommand interrupted
   */
  @Override
  public void end(boolean interrupted) {
    
    if (!interrupted) {
      GameState.getInstance().setGamePieceHeld(true);
    }
    m_intakeSubsystem.hold();
    timeout.stop();
  }

  /**
   *  Returns true when motor stall timeout reaches 0.5 seconds.
   */ 
  @Override
  public boolean isFinished() {
    return timeout.hasElapsed(0.1);
  }
}
