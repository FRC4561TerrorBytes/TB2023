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
import frc.robot.commands.GroundIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class ConeHighBalance extends BasePathAuto{

  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in autonomous
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public ConeHighBalance(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem, IntakeSubsystem intakeSubsystem, String autoPathName, double maxSpeedMetersPerSec,
  double maxAccelerationMetersPerSecSquared) {

    super(driveSubsystem, autoPathName, maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared);

    this.m_armSubsystem = armsubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
  }
}