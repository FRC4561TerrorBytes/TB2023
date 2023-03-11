// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ScoreAlign extends CommandBase {
  /** Creates a new AutoRotate. */

  DriveSubsystem m_driveSubsystem;
  VisionSubsystem m_visionSubsystem;

  Pose2d startPosition;
  double rotateTo = 180.0;
  double rotateTolerance = 0.1;

  double startingAngle;

  public ScoreAlign(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    addRequirements(m_driveSubsystem);
    addRequirements(m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingAngle = m_driveSubsystem.getPose().getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = m_driveSubsystem.getPose().getRotation().getDegrees();
    double angle = rawAngle%360;
    double calculatedAngle = rotateTo - angle;

    m_driveSubsystem.drive(0, 0, 1.3 * Math.signum(angle), false);

    System.out.println("raw angle: " + rawAngle);
    System.out.println("angle: " + angle);
    System.out.println("calculatedAngle: " + calculatedAngle);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("target Rotation: " + rotateTo);
    if(Math.signum(startingAngle) != Math.signum(m_driveSubsystem.getPose().getRotation().getDegrees())){
      // if(Math.abs((m_driveSubsystem.getPose().getRotation().getDegrees()%360)) >= rotateTo + rotateTolerance){
        System.out.println("rotation has finished");
        // m_driveSubsystem.drive(0, 0, 1 * Math.signum(), false);
        if (Math.abs(m_driveSubsystem.getPose().getRotation().getDegrees()%360 - rotateTo) >  rotateTolerance) { //add to if above

        }
        return true;
      }
      else{
        System.out.println("rotation has not finished");
        return false;
      }
    else{
      System.out.println("rotation has not finished");
      return false;
    }
  }
}
