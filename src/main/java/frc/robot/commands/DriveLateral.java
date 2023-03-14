// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveLateral extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private double m_startingY;
  private double m_distance;

  private PIDController m_controller = new PIDController(5, 0.0, 0.0);

  /** Creates a new DriveDistance. */
  public DriveLateral(DriveSubsystem driveSubsystem, double distance, double tolerance) {
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;
    m_controller.setSetpoint(distance);
    m_controller.setTolerance(tolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
    m_startingY = m_driveSubsystem.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lateralMoveSpeed = m_controller.calculate(m_driveSubsystem.getPose().getY(), m_startingY + m_distance);
    System.out.println("thing we give calculate: " + ( m_startingY + m_distance));
    System.out.println("lateral speed: " + lateralMoveSpeed);
    m_driveSubsystem.drive(0, lateralMoveSpeed, 0, false);
    SmartDashboard.putBoolean("driving forward", true);
    System.out.println("finished: " + m_controller.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
