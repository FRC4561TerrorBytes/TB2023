// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.KnownArmPlacement;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveConeMiddleCommand extends SequentialCommandGroup {
  /** Creates a new MoveConeScoreHighCommand. */
  public MoveConeMiddleCommand(ArmSubsystem m_armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        //new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SUBSTATION_APPROACH)),
        //new WaitCommand(1.0),
        new InstantCommand(() -> m_armSubsystem.setKnownArmPlacement(KnownArmPlacement.SCORE_CONE_MIDDLE))
    );
  }
}
