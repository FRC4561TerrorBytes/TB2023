package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
  private CANSparkMax m_elbowMotor = new CANSparkMax(13, MotorType.kBrushless);
  private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);
  private double zeroShoulderPosition;
  private boolean changing = true;
  private double targetPosition = 0;

  public ArmSubsystem() {
    m_shoulderMotor.configFactoryDefault();
    m_elbowMotor.setSmartCurrentLimit(2);
    m_shoulderMotor.config_kP(0, 0.005);
    m_shoulderMotor.set(ControlMode.Position, 0);
    zeroShoulderPosition = m_shoulderMotor.getSelectedSensorPosition();
  }
    public void resetPosition(){
      System.out.println("reset");
      m_shoulderMotor.set(ControlMode.Position, zeroShoulderPosition);
    }

    public void setArmSpeed(double shoulderSpeed, double elbowSpeed){
      m_shoulderMotor.set(ControlMode.PercentOutput, shoulderSpeed);
      m_elbowMotor.set(elbowSpeed);            
      if (shoulderSpeed == 0 && changing == true){
        changing = false;
        targetPosition = m_shoulderMotor.getSelectedSensorPosition();
      }
      else if (shoulderSpeed != 0 && changing == false){
        changing = true;
      }
      else if (shoulderSpeed == 0 && changing == false){
        m_shoulderMotor.set(ControlMode.Position, targetPosition);
      }
    }
}
