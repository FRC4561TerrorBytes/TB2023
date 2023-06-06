// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveConeHighCommand;
import frc.robot.commands.ScoreAutoCube;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.GroundIntake;
public class TestMirroring {

  DriveSubsystem m_driveSubsystem;
  PathPlannerTrajectory m_pathPlannerTrajectory;
  PPSwerveControllerCommand m_swerveControllerCommand;
  HashMap<String, Command> m_eventMap = new HashMap<>();
  String autoPathName = "";
  PathPlannerTrajectory transformedTrajectory;
  boolean isRedAlliance;

  double maxSpeed;
  double maxAccel;

    /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in autonomous
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public TestMirroring(DriveSubsystem driveSubsystem, String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared, boolean isRedAlliance) {
    this.m_driveSubsystem = driveSubsystem;
    this.maxSpeed = maxSpeedMetersPerSec;
    this.maxAccel = maxAccelerationMetersPerSecSquared;

    m_pathPlannerTrajectory = PathPlanner.loadPath(autoPathName, maxSpeedMetersPerSec,
        maxAccelerationMetersPerSecSquared);

    transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(m_pathPlannerTrajectory, DriverStation.getAlliance());
    
    this.autoPathName = autoPathName;
    this.isRedAlliance = isRedAlliance;
    
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry(transformedTrajectory.getInitialHolonomicPose());
  }

  public Command getCommandAndStop() {
    return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(new FollowPathWithEvents(new AutoTrajectory(m_driveSubsystem,autoPathName, maxSpeed, maxAccel, isRedAlliance).getCommandAndStop(), transformedTrajectory.getMarkers(), m_eventMap))
            .andThen(() -> m_driveSubsystem.stop());
  }
}