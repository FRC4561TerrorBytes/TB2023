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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
public class LowLinkRIGHT {

  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_ArmSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  PathPlannerTrajectory m_pathPlannerTrajectory;
  PPSwerveControllerCommand m_swerveControllerCommand;
  HashMap<String, Command> m_eventMap = new HashMap<>();
  String autoPathName = "";
  boolean isRedAlliance;

    /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in autonomous
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public LowLinkRIGHT(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem, IntakeSubsystem intakeSubsystem, String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared, boolean isRedAlliance) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_ArmSubsystem = armsubsystem;
    this.m_IntakeSubsystem = intakeSubsystem;

    m_pathPlannerTrajectory = PathPlanner.loadPath(autoPathName, maxSpeedMetersPerSec,
        maxAccelerationMetersPerSecSquared);

    // m_eventMap Goes Here
    m_eventMap.put("ScoreCube1", new ScheduleCommand(new IntakeCommand(intakeSubsystem).withTimeout(1)));
    m_eventMap.put("Stow1", new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));

    this.autoPathName = autoPathName;
    this.isRedAlliance = isRedAlliance;
  
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry(m_pathPlannerTrajectory.getInitialHolonomicPose());
  }

  public Command getCommandAndStop() {
    return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(new FollowPathWithEvents(new AutoTrajectory(m_driveSubsystem,autoPathName,3,2, isRedAlliance).getCommandAndStop(), m_pathPlannerTrajectory.getMarkers(), m_eventMap))
            .andThen(() -> m_driveSubsystem.stop());
  }
}