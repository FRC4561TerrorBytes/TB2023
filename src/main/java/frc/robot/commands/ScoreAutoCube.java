// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreAutoCube extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new ScoreCommand. */
  public ScoreAutoCube(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  //we score cubes but we are in STOWED so we outtake faster(cone outtake speed).
  @Override
  public void execute() {
    m_intakeSubsystem.setRollerSpeed(Constants.SCORE_SPEED_CONE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GameState.getInstance().setGamePieceHeld(false);
    m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_HOLD_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
