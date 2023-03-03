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

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftIntakeMotor = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightIntakeMotor = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder m_leftEncoder = m_leftIntakeMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightIntakeMotor.getEncoder();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(true);
    m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);

  }

  public void setIntakeSpeed(double speed) {
    m_leftIntakeMotor.set(speed);
    m_rightIntakeMotor.set(speed);
  }

  public void intake() {
    setIntakeSpeed(Constants.INTAKE_SPEED);
  }

  public void hold() {
    setIntakeSpeed(Constants.INTAKE_HOLD_SPEED);
  }

  public void score() {
    setIntakeSpeed(Constants.INTAKE_SCORE_SPEEED);
  }

  public void scoreConeMiddle() {
    setIntakeSpeed(Constants.INTAKE_CONE_MIDDLE_SPEED);
  }

  public void scoreConeHigh() {
    setIntakeSpeed(Constants.INTAKE_CONE_HIGH_SPEED);
  }

  public void stop() {
    setIntakeSpeed(0.0);
  }

  public boolean isStalled() {
    if (this.getCurrentCommand().getName().endsWith("IntakeCommand")) {
      return m_leftEncoder.getVelocity() < 100.0 && m_rightEncoder.getVelocity() < 100.0;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left intake current", m_leftIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left intake velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right intake current", m_rightIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right intake velocity", m_rightEncoder.getVelocity());
  }
}
