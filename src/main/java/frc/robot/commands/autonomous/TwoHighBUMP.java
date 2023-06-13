// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TwoHighBUMP extends BasePathAuto {

  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  PathPlannerTrajectory m_pathPlannerTrajectory;
  PPSwerveControllerCommand m_swerveControllerCommand;
  String autoPathName = "";
  PathPlannerTrajectory transformedTrajectory;
  boolean isRedAlliance;

  /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in
   * autonomous
   * 
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public TwoHighBUMP(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem, IntakeSubsystem intakeSubsystem,
      String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared) {

    super(driveSubsystem, autoPathName, maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared);

    this.m_armSubsystem = armsubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    HashMap<String, Command> eventMap = new HashMap<>(); 
    // Approach then stow
    eventMap.put("Approach1",
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)));
    eventMap.put("Stow1", new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));

    eventMap.put("GameStateChange1",
        new InstantCommand(() -> GameState.getInstance().setGamePieceDesired(GamePiece.CUBE)));

    // going to floor grab and intaking
    eventMap.put("goToFloor1",
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CUBE)));
    eventMap.put("intake1", new ScheduleCommand(new IntakeCommand(m_intakeSubsystem)));

    // going back to stow to move arm out of the way
    eventMap.put("Approach2",
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)));
    eventMap.put("Print1", new InstantCommand(() -> System.out.println("OIDUABGAYGDWVBI \n \n \n")));

    // Approach to high for score
    eventMap.put("High1",
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CUBE_HIGH)));

    // Score cube high
    eventMap.put("ScoreCube2", new ScheduleCommand(new ScoreCommand(intakeSubsystem).withTimeout(0.5)));

    setEventMap(eventMap);

  }
}