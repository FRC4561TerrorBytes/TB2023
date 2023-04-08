// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroWristCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private int isZeroed;

  /** Creates a new ZeroWristCommand. */
  public ZeroWristCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isZeroed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_armSubsystem.setManualWristSpeed(-0.1);
  
    if(m_armSubsystem.getWristThroughboreEncPosition() < Constants.WRIST_ENCODER_ZERO_THRESHOLD) {
      isZeroed++;
      if (isZeroed >= 4){
        m_armSubsystem.resetWristPosition();
        m_armSubsystem.setManualWristSpeed(0.0);
      }
    }
    else{
      isZeroed = 0;
    }

    SmartDashboard.putNumber("Wrist Stalls", isZeroed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setManualWristSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isZeroed >= 4;
  }
}