// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArmCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private double shoulderSpeed, elbowSpeed;
  private int shoulderLimitContacts, elbowLimitContacts;

  /** Creates a new ResetArmCommand. */
  public ResetArmCommand(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderSpeed = -0.1;
    elbowSpeed = -0.05;
    shoulderLimitContacts = 0;
    elbowLimitContacts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_armSubsystem.setArmDifferential(0.1, 0);
    //m_armSubsystem.proceedToArmPosition();
    if (m_armSubsystem.elbowLimitReached() == true){
      elbowLimitContacts++;
      if (elbowLimitContacts == 4){
        m_armSubsystem.resetElbowPosition();
        elbowSpeed = 0;
      }
    }
    if (m_armSubsystem.shoulderLimitReached() == true){
      shoulderLimitContacts++;
      if (shoulderLimitContacts == 4){
        m_armSubsystem.resetShoulderPosition();
        shoulderSpeed = 0;
      }
    }
    m_armSubsystem.setArmSpeed(shoulderSpeed, elbowSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSpeed = 0;
    elbowSpeed = 0;
    m_armSubsystem.setArmSpeed(shoulderSpeed, elbowSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elbowLimitContacts >= 4 && shoulderLimitContacts >= 4;
  }
}
