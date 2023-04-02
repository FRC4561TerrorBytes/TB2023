// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.ConnectException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

public class GroundIntake extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ArmSubsystem m_armSubsystem;
  private Timer timeout = new Timer();
  private Timer timer = new Timer();

  /** Creates a new IntakeCommand. */
  public GroundIntake(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_armSubsystem = armSubsystem;

    addRequirements(m_intakeSubsystem);
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {
    timeout.reset();
    timer.reset();
    timer.start();
    m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CUBE);
    m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_SPEED);
  }

  /**
   * Run Intake at speed until motors stall (game piece acquired)
   */
  @Override
  public void execute() {
    m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_SPEED);
    if (m_intakeSubsystem.isStalled() && timer.hasElapsed(0.25)) {
      timeout.start();
    }

    SmartDashboard.putNumber("Timeout", timer.get());
  }

  /**
   * Set Game State to Held when IntakeCommand interrupted
   */
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      GameState.getInstance().setGamePieceHeld(true);
    }
    
    SmartDashboard.putBoolean("Interrupted", interrupted);
    m_intakeSubsystem.setRollerSpeed(Constants.INTAKE_HOLD_SPEED);
    m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED);
    timeout.stop();
    timer.stop();
  }

  /**
   *  Returns true when motor stall timeout reaches 0.1 seconds.
   */ 
  @Override
  public boolean isFinished() {
    return (timeout.hasElapsed(0.1) || timer.hasElapsed(2.0));
  }
}
