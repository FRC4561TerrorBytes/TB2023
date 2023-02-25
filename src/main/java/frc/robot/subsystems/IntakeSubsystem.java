// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_leftIntakeMotor = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless); // front sensor
  private CANSparkMax m_rightIntakeMotor = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless); // back sensor

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(true);
    m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntakeSpeed(double speed){
    m_leftIntakeMotor.set(speed);
    m_rightIntakeMotor.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
