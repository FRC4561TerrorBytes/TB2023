// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ScoreAlign extends CommandBase {
  /** Creates a new AutoRotate. */

  final DriveSubsystem m_driveSubsystem;
  final PIDController m_pidController = new PIDController(0.03, 0, 0);

  public ScoreAlign(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
    m_pidController.enableContinuousInput(-180.0, 180.0);
    //m_pidController.setSetpoint(0.0);
    m_pidController.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
    //final double absAngle = Math.abs(m_driveSubsystem.getPose().getRotation().getDegrees());
    final double angle = m_driveSubsystem.getPose().getRotation().getDegrees() + 180;
    int degreesClosestTo = 0;
    double closest = 999.0;
    
    if(Math.abs(angle - 360) < closest){
      closest = Math.abs(angle - 360);
      degreesClosestTo = 360;
    }
    if(Math.abs(angle - 270) < closest){
      closest = Math.abs(angle - 270);
      degreesClosestTo = 270;
    }
    if(Math.abs(angle - 180) < closest){
      closest = Math.abs(angle - 180);
      degreesClosestTo = 180;
    }
    if(Math.abs(angle - 90) < closest){
      closest = Math.abs(angle - 90);
      degreesClosestTo = 90;
    }
    if(Math.abs(angle + 0) < closest){
      closest = Math.abs(angle + 0);
      degreesClosestTo = 0;
    }
    // if(Math.abs(angle - 180) < closest){
    //   closest = Math.abs(angle - 180);
    //   degreesClosestTo = 180;
    // }
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("rotating to", degreesClosestTo);
    m_pidController.setSetpoint(degreesClosestTo);

    //final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    //m_pidController.setSetpoint(closerTo0 ? 0.0 : 180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = m_driveSubsystem.getPose().getRotation().getDegrees();
    double rotationRate = m_pidController.calculate(rawAngle + 180);    
    rotationRate += 1.2 * Math.signum(rotationRate);
    System.out.println(m_driveSubsystem.getPose().getRotation().getDegrees() + 180);

    m_driveSubsystem.drive(0, 0, rotationRate, true);

    System.out.println("rotation from pose: " + (m_driveSubsystem.getPose().getRotation().getDegrees() + 180));
    // System.out.println("rotation from pigeon: " + m_driveSubsystem.getPigeonYaw());

    SmartDashboard.putNumber("Raw Angle", rawAngle);
    SmartDashboard.putNumber("Rotation Rate", rotationRate);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}