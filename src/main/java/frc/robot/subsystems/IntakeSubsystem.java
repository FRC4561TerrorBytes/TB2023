// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_leftIntakeMotor = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless); // front sensor
  private CANSparkMax m_rightIntakeMotor = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless); // back sensor
  private SparkMaxLimitSwitch m_frontLimit;
  private SparkMaxLimitSwitch m_backLimit;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(true);
    m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);

    m_frontLimit = m_leftIntakeMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_backLimit = m_rightIntakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
  }

  public boolean isFrontLimitBroken() {
    return m_frontLimit.isPressed();
  }

  public boolean isBackLimitBroken() {
    return m_backLimit.isPressed();
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

  public void stop() {
    setIntakeSpeed(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Front Intake Limit", isFrontLimitBroken());
    SmartDashboard.putBoolean("Back Intake Limit", isBackLimitBroken());
  }
}
