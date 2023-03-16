// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroShoulderCommand extends CommandBase {
  /** Creates a new ZeroShoulderCommand. */
  private ArmSubsystem m_armSubsystem;
  private int shoulderLimitContacts;

  public ZeroShoulderCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderLimitContacts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.shoulderLimitReached() == true) {
      shoulderLimitContacts++;
      if (shoulderLimitContacts == 4) {
        m_armSubsystem.resetShoulderPosition();
      }
    } else {
      // shoulderLimitContacts = 0;
    }
    m_armSubsystem.setManualShoulderSpeed(0.2);

    SmartDashboard.putNumber("Shoulder Contacts", shoulderLimitContacts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setManualShoulderSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoulderLimitContacts >= 4;
  }
}
