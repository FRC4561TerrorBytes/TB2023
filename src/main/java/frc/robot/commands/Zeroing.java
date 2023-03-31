// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;

public class Zeroing extends CommandBase {
  /** Creates a new Zeroing. */
  private ArmSubsystem m_armSubsystem;
  private int elbowLimitContacts;
  private int shoulderLimitContacts;
  private int wristIsStalled;

  public Zeroing(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowLimitContacts = 0;
    shoulderLimitContacts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_armSubsystem.elbowLimitReached() == true) {
      elbowLimitContacts++;
    } else {
      elbowLimitContacts = 0;
    }
    if (elbowLimitContacts >= 4) {
      m_armSubsystem.resetElbowPosition();
    } else {
      m_armSubsystem.setManualElbowSpeed(-0.15);
    }

    // zero shoulder
    if (m_armSubsystem.shoulderLimitReached() == true) {
      shoulderLimitContacts++;
    } else {
      shoulderLimitContacts = 0;
    }
    if (shoulderLimitContacts >= 4) {
      m_armSubsystem.resetShoulderPosition();
    } else {
      m_armSubsystem.setManualShoulderSpeed(0.2);
    }

    if (m_armSubsystem.isWristStalled()) {
      wristIsStalled++;
    } else {
      wristIsStalled = 0;
    }
    if (wristIsStalled >= 4) {
      m_armSubsystem.resetWristPosition();
      m_armSubsystem.setManualWristSpeed(0.0);
    } else {
      m_armSubsystem.setManualWristSpeed(-0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setManualElbowSpeed(0);
    m_armSubsystem.setManualShoulderSpeed(0);
    m_armSubsystem.setManualWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elbowLimitContacts >= 4) && (shoulderLimitContacts >= 4) && (wristIsStalled >= 4);
  }
}
