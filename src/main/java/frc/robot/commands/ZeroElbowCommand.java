// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroElbowCommand extends CommandBase {
  /** Creates a new ZeroElbowCommand. */
  private ArmSubsystem m_armSubsystem;
  private int elbowLimitContacts;

  public ZeroElbowCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbowLimitContacts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.elbowLimitReached() == true) {
      elbowLimitContacts++;
      if (elbowLimitContacts == 4) {
        m_armSubsystem.resetElbowPosition();
      }
    } else {
      // elbowLimitContacts = 0;
    }
    m_armSubsystem.setElbowSpeed(0.0);

    SmartDashboard.putNumber("Elbow Contacts", elbowLimitContacts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setElbowSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elbowLimitContacts >= 4;
  }
}
