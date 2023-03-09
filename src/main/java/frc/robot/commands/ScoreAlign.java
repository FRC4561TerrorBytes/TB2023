// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ScoreAlign extends CommandBase {
  /** Creates a new AutoRotate. */

  DriveSubsystem m_driveSubsystem;

  Pose2d startPosition;
  double rotateTo = 180.0;
  double rotateTolerance = 0.5;

  public ScoreAlign(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(0, 0, rotateTo - m_driveSubsystem.getPose().getRotation().getDegrees(), false);
    System.out.println("rotation speed: " + (rotateTo - m_driveSubsystem.getPose().getRotation().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_driveSubsystem.getPose().getRotation().getDegrees() <= rotateTo + rotateTolerance || m_driveSubsystem.getPose().getRotation().getDegrees() >= rotateTo - rotateTolerance){
      System.out.println("rotation has finished");
      return true;
    }
    else{
      System.out.println("rotation has not finished");
      return false;
    }
  }
}
