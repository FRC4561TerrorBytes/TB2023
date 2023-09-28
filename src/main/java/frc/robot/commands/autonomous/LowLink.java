// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class LowLink extends BasePathAuto{

  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;

    /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in autonomous
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public LowLink(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem, IntakeSubsystem intakeSubsystem, String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared) {
      
    super(driveSubsystem, autoPathName, maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared);

    this.m_armSubsystem = armsubsystem;
    this.m_intakeSubsystem = intakeSubsystem;

    //scoring first piece
    m_eventMap.put("score1", new ScheduleCommand(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5)));
    //going to floor grab and intaking
    m_eventMap.put("goToFloor1", new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CUBE)));
    m_eventMap.put("intake1", new ScheduleCommand(new IntakeCommand(m_intakeSubsystem).withTimeout(0.5)));
    
    //going back to stow to move arm out of the way
    m_eventMap.put("Stow1", new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));
    m_eventMap.put("score2", new ScheduleCommand(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5)));

    m_eventMap.put("goToFloor2", new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.FLOOR_GRAB_CUBE)));
    m_eventMap.put("intake2",  new ScheduleCommand(new IntakeCommand(m_intakeSubsystem).withTimeout(0.5)));
    m_eventMap.put("Stow2", new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.STOWED)));

    m_eventMap.put("score3", new ScheduleCommand(new ScoreCommand(m_intakeSubsystem).withTimeout(0.5)));

    setEventMap(m_eventMap);
  }
}