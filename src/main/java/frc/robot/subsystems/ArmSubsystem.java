package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
// private WPI_TalonFX m_elbowMotor = new WPI_TalonFX(4);
// private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);
  private CANSparkMax m_shoulderMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_elbowMotor = new CANSparkMax(11, MotorType.kBrushless);
 private SparkMaxPIDController m_shoulderController;
 private SparkMaxPIDController m_elbowController;


  private double m_targetShoulderPosition = 0.0;
  private double m_targetElbowPosition = 0.0;

  /**
   * An enumeration of known arm placements, e.g. stowed or score cone high). The
   * angles are in degrees from horizontal when the controlled arm segment is
   * pointing forward. We use horizontal as 0 for our degree measurements rather
   * than vertical due to how cosine and standard trigonometry angle measurements
   * work.
   */
  public enum KnownArmPlacement {
    STOWED(90.0, -90.0),
    HOME(75.0, -75.0),
    TEMP_ELBOW_HALFWAY(90.0, -45.0),
    TEMP_ELBOW_HORT(90.0, 0),
    TEMP_ELBOW_PAST_HORT_BY_HALF(90.0, 45.0),
    TEMP_STRETCHED_PAST_HORT_BY_HALF(45.0, 45.0),
    TEMP_REACHED_OUT(45.0, 0.0);
    //TEMP_ELBOW_PAST_TOP_BY_HALF(90.0, 135.0);

    public final double m_shoulderAngle;
    public final double m_elbowAngle;

    private KnownArmPlacement(double shoulderAngle, double elbowAngle) {
      m_shoulderAngle = shoulderAngle;
      m_elbowAngle = elbowAngle;
    }
  }

  public ArmSubsystem() {
    m_shoulderController = m_shoulderMotor.getPIDController();
    m_elbowController = m_elbowMotor.getPIDController();
    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.restoreFactoryDefaults();
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_elbowController.setReference(0, ControlType.kSmartMotion);
    /*final NeoSteerConfiguration elbowConfig = new NeoSteerConfiguration<EncoderConfiguration>(11, null)
    elbowConfig.neutralDeadband = ArmConstants.ARM_MOTORS_NEUTRAL_DEADBAND;
    elbowConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    elbowConfig.slot0.kP = ArmConstants.ELBOW_PROPORTIONAL_GAIN;
    elbowConfig.slot0.integralZone = ArmConstants.ARM_MOTORS_INTERGRAL_ZONE;
    elbowConfig.slot0.closedLoopPeakOutput = ArmConstants.ARM_MOTORS_CLOSED_LOOP_PEAK_OUTPUT;
    elbowConfig.slot0.allowableClosedloopError = ArmConstants.ELBOW_TOLERANCE;
    elbowConfig.motionAcceleration = ArmConstants.ELBOW_PEAK_ACCELERATION;
    elbowConfig.motionCruiseVelocity = ArmConstants.ELBOW_CRUISE_VELOCITY;
    elbowConfig.forwardSoftLimitEnable = true;
    elbowConfig.forwardSoftLimitThreshold = 0;
    elbowConfig.reverseSoftLimitEnable = true;
    elbowConfig.reverseSoftLimitThreshold = ArmConstants.ELBOW_ONLY_HORIZONTAL_TICKS * 2.0;
    m_elbowMotor.configAllSettings(elbowConfig);
    final TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
    shoulderConfig.neutralDeadband = ArmConstants.ARM_MOTORS_NEUTRAL_DEADBAND;
    shoulderConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    shoulderConfig.slot0.kP = ArmConstants.SHOULDER_PROPORTIONAL_GAIN;
    shoulderConfig.slot0.integralZone = ArmConstants.ARM_MOTORS_INTERGRAL_ZONE;
    shoulderConfig.slot0.closedLoopPeakOutput = ArmConstants.ARM_MOTORS_CLOSED_LOOP_PEAK_OUTPUT;
    shoulderConfig.slot0.allowableClosedloopError = ArmConstants.SHOULDER_TOLERANCE;
    shoulderConfig.motionAcceleration = ArmConstants.SHOULDER_PEAK_ACCELERATION;
    shoulderConfig.motionCruiseVelocity = ArmConstants.SHOULDER_CRUISE_VELOCITY;
    shoulderConfig.forwardSoftLimitEnable = true;
    shoulderConfig.forwardSoftLimitThreshold = 0;
    shoulderConfig.reverseSoftLimitEnable = true;
    shoulderConfig.reverseSoftLimitThreshold = ArmConstants.SHOULDER_ONLY_HORIZONTAL_TICKS * 2.0;
    m_shoulderMotor.configAllSettings(shoulderConfig);*/
  }

  public void resetPosition() {
    System.out.println("reset");
    m_shoulderMotor.set(m_targetShoulderPosition);
    m_elbowMotor.set(m_targetElbowPosition);
  }

  public void setArmSpeed(double shoulderSpeed, double elbowSpeed) {
    m_shoulderMotor.set(shoulderSpeed);
    m_elbowMotor.set(elbowSpeed);
  }

  /**
   * @param placement the desired updated arm placement.
   */
  public void setKnownArmPlacement(KnownArmPlacement placement) {
    // TODO convert shoulder angle to encoder clicks.
    double desiredShoulderAngle = placement.m_shoulderAngle;
    double desiredShoulderTicks = desiredShoulderAngle * ArmConstants.SHOULDER_TICKS_PER_DEGREE + ArmConstants.SHOULDER_ONLY_HORIZONTAL_TICKS;
    double desiredElbowAngle = placement.m_elbowAngle - (placement.m_shoulderAngle - 90.0);
    double desiredElbowTicks = desiredElbowAngle * ArmConstants.ELBOW_TICKS_PER_DEGREE
        + ArmConstants.ELBOW_ONLY_HORIZONTAL_TICKS;
    setShoulderPosition(desiredShoulderTicks);    
    setElbowPosition(desiredElbowTicks);
  }

  /**
   * @param targetPosition the target position in encoder ticks.
   */
  void setShoulderPosition(double targetPosition) {
    m_targetShoulderPosition = targetPosition;
  }

  /**
   * @param targetPosition the target position in encoder ticks.
   */
  void setElbowPosition(double targetPosition) {
    m_targetElbowPosition = targetPosition;
  }

  public void proceedToArmPosition() {
    proceedToShoulderPosition();
    proceedToElbowPosition();
  }

  /**
   * Drives the elbow toward the last postion set via
   * {@link} {@link #setElbowPosition(double)}}.
   */
  void proceedToElbowPosition() {
    /*double currentPos = m_elbowMotor.getSelectedSensorPosition();
    double degrees = (currentPos - ArmConstants.ELBOW_ONLY_HORIZONTAL_TICKS) / ArmConstants.ELBOW_TICKS_PER_DEGREE;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_elbowMotor.set(
        ControlMode.MotionMagic, m_targetElbowPosition,
        DemandType.ArbitraryFeedForward, ArmConstants.ELBOW_MAX_VOLTAGE_FF * cosineScalar);*/
  }

  void proceedToShoulderPosition() {
    /*double currentPos = m_shoulderMotor.getSelectedSensorPosition();
    double degrees = (currentPos - ArmConstants.SHOULDER_ONLY_HORIZONTAL_TICKS) / ArmConstants.SHOULDER_TICKS_PER_DEGREE;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_shoulderMotor.set(
        ControlMode.MotionMagic, m_targetShoulderPosition,
        DemandType.ArbitraryFeedForward, ArmConstants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar);*/
  }

  @Override
  public void periodic() {
    /*SmartDashboard.putNumber("Elbow encoder", m_elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Elbow current", m_elbowMotor.getStatorCurrent());
    SmartDashboard.putNumber("Shoulder encoder", m_shoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder voltage", m_shoulderMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Shoulder current", m_shoulderMotor.getStatorCurrent());*/
  }
}
