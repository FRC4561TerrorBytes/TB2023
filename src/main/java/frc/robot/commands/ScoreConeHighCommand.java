// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GameState;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreConeHighCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new ScoreCommand. */
  public ScoreConeHighCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.scoreConeHigh();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameState.getInstance().setGamePieceHeld(false);
    m_intakeSubsystem.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}