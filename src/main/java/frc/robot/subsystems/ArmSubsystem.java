package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(11);
    private WPI_TalonFX m_elbowMotor = new WPI_TalonFX(12);

    public void stopMotor(){
      m_shoulderMotor.stopMotor();
      m_elbowMotor.stopMotor();

    }

    public void setElbowSpeed(double speed){
      m_elbowMotor.set(ControlMode.PercentOutput, speed);
    }
}
