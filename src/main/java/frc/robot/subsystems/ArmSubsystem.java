package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
  public CANSparkMax m_elbowMotor = new CANSparkMax(13, MotorType.kBrushless);
  public WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);

  public ArmSubsystem() {
    m_shoulderMotor.configFactoryDefault();
    m_elbowMotor.setSmartCurrentLimit(2);
  }
    public void stopMotor(){
      m_shoulderMotor.stopMotor();

    }

    public void setArmSpeed(double shoulderSpeed, double elbowSpeed){
      m_shoulderMotor.set(ControlMode.PercentOutput, shoulderSpeed);
      m_elbowMotor.set(elbowSpeed);
    }
}
