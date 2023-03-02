// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private Pose2d m_startingPose;
  private double m_speed;
  private double m_distance;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem driveSubsystem, double distance, double speed) {
    m_driveSubsystem = driveSubsystem;
    m_speed = speed;
    m_distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingPose = m_driveSubsystem.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(m_speed, 0, 0, false);
    SmartDashboard.putBoolean("driving forward", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_startingPose.getX() + m_distance < m_driveSubsystem.getPose().getX();
  }
}
