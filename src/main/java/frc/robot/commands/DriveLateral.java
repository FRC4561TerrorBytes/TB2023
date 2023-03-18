// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveLateral extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final double m_distance;
  private final PIDController m_controller = new PIDController(2.0, 0.0, 0.0);

  /** Creates a new DriveDistance. */
  public DriveLateral(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double distance, double tolerance){
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_distance = distance;
    m_controller.setTolerance(tolerance);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
    final double startingY = m_driveSubsystem.getPose().getY();
    final double setPoint = startingY + m_visionSubsystem.getLateralDistance(m_distance);
    m_controller.setSetpoint(setPoint);
    SmartDashboard.putNumber("Lat init strtY", startingY);
    SmartDashboard.putNumber("Lat init dist", m_distance);
    SmartDashboard.putNumber("Lat init setPt", setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute is running");
    final double currentY = m_driveSubsystem.getPose().getY();
    double pidSpeed = m_controller.calculate(currentY);
    double lateralMoveSpeed = pidSpeed + (0.25 * Math.signum(pidSpeed));
    SmartDashboard.putNumber("Lat exe curY", currentY);
    SmartDashboard.putNumber("Lat exe pidSp", pidSpeed);
    SmartDashboard.putNumber("Lat exe speed", lateralMoveSpeed);
    m_driveSubsystem.drive(0, lateralMoveSpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Lat estFini", m_controller.atSetpoint());
    return m_controller.atSetpoint();
  }
}
