package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  private WPI_TalonFX m_elbowMotor = new WPI_TalonFX(4);
  private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);

  public ArmSubsystem() {
    m_shoulderMotor.configFactoryDefault();
    m_elbowMotor.configFactoryDefault();
    m_shoulderMotor.config_kP(0, 0.005);
    m_shoulderMotor.set(ControlMode.Position, 0);
  }
    public void resetPosition(){
      System.out.println("reset");
      m_shoulderMotor.setSelectedSensorPosition(0.0);
      m_elbowMotor.setSelectedSensorPosition(0.0);
    }

    public void setArmSpeed(double shoulderSpeed, double elbowSpeed){
      m_shoulderMotor.set(ControlMode.PercentOutput, shoulderSpeed);
      m_elbowMotor.set(ControlMode.PercentOutput, elbowSpeed);            
    }
  
  public void setElbowPosition(double encoderTarget) {
    // double forearmAngle = m_elbowMotor.getSelectedSensorPosition(); // TODO map encoder to angle
    // double gravityFeedForward = forearmAngle; //TODO map angle to ff
    double gravityFeedForward = m_elbowMotor.getSelectedSensorPosition() * 0.0000035;
    m_elbowMotor.set(ControlMode.MotionMagic, encoderTarget, DemandType.ArbitraryFeedForward, gravityFeedForward);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow encoder", m_elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Elbow current", m_elbowMotor.getStatorCurrent());
  }
}
