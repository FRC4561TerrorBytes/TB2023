// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreAutoCube;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class LowLink {

  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_ArmSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  PathPlannerTrajectory m_pathPlannerTrajectory;
  PPSwerveControllerCommand m_swerveControllerCommand;
  HashMap<String, Command> m_eventMap = new HashMap<>();
  String autoPathName = "";

    /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in autonomous
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public LowLink(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem, IntakeSubsystem intakeSubsystem, String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_ArmSubsystem = armsubsystem;
    this.m_IntakeSubsystem = intakeSubsystem;

    m_pathPlannerTrajectory = PathPlanner.loadPath(autoPathName, maxSpeedMetersPerSec,
        maxAccelerationMetersPerSecSquared);

    m_eventMap.put("cubeScore1", new ScheduleCommand(new ScoreAutoCube(m_IntakeSubsystem).withTimeout(0.5)));
    m_eventMap.put("eventGroundIntake1", new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB)));
    m_eventMap.put("Stow1", new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_eventMap.put("scoreLow2",  new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_LOW_CUBE)));
    m_eventMap.put("scoreCubeMiddle",  new ScheduleCommand(new ScoreAutoCube(m_IntakeSubsystem).withTimeout(0.5)));
    m_eventMap.put("eventGroundIntake2", new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB)));
    m_eventMap.put("scoreLow3",  new InstantCommand(() -> m_ArmSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_LOW_CUBE)));
    m_eventMap.put("scoreCubeRight",  new ScheduleCommand(new ScoreAutoCube(m_IntakeSubsystem).withTimeout(0.5)));

    this.autoPathName = autoPathName;
    // Auto PID Controllers
    PIDController xController = new PIDController(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD);
    PIDController yController = new PIDController(Constants.AUTO_Y_KP, Constants.AUTO_Y_KI, Constants.AUTO_Y_KD);
    PIDController thetaController = new PIDController(Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI,
        Constants.AUTO_THETA_KD);
    //FIXME: TRY WITHOUT CONTINUOUS INPUT
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_swerveControllerCommand = new PPSwerveControllerCommand(
        m_pathPlannerTrajectory,
        m_driveSubsystem::getPose,
        Constants.DRIVE_KINEMATICS,
        xController,
        yController,
        thetaController,
        m_driveSubsystem::setModuleStates,
        m_driveSubsystem);
  }

  public void resetOdometry() {
    m_driveSubsystem.resetOdometry(m_pathPlannerTrajectory.getInitialHolonomicPose());
  }

  public Command getCommandAndStop() {
    return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(new FollowPathWithEvents(new AutoTrajectory(m_driveSubsystem,autoPathName,3,2).getCommandAndStop(), m_pathPlannerTrajectory.getMarkers(), m_eventMap))
            .andThen(() -> m_driveSubsystem.stop());
  }
}