// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.ROLLER_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.setInverted(true);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setRollerSpeed(double speed) {
    double sign = 1.0;
    if (GameState.getInstance().getGamePieceDesired() == GamePiece.CONE) {
      sign = -1.0;
    }
    m_intakeMotor.set(sign * speed);
  }

  public void stop() {
    setRollerSpeed(0.0);
  }

  public boolean isStalled() {
    return Math.abs(m_intakeEncoder.getVelocity()) < 100.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left intake current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left intake velocity", m_intakeEncoder.getVelocity());
  }
}
