package frc.robot.subsystems;

import javax.sound.sampled.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
// private WPI_TalonFX m_elbowMotor = new WPI_TalonFX(4);
// private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);
  private CANSparkMax m_shoulderMotor = new CANSparkMax(12, MotorType.kBrushless);  
  private RelativeEncoder m_shoulderEncoder; 
  private SparkMaxPIDController m_shoulderController;
  private CANSparkMax m_elbowMotor = new CANSparkMax(11, MotorType.kBrushless);
  private RelativeEncoder m_elbowEncoder;
  private SparkMaxPIDController m_elbowController;


  private double m_targetShoulderPosition = 0.0;
  private double m_targetElbowPosition = 0.0;

  private boolean m_prototypeArm = true;

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
    m_elbowMotor.restoreFactoryDefaults();
    m_elbowController = m_elbowMotor.getPIDController();
    m_elbowEncoder = m_elbowMotor.getEncoder();    
    m_elbowMotor.setIdleMode(IdleMode.kBrake);  
    m_elbowController.setP(ArmConstants.ELBOW_PROPORTIONAL_GAIN);
    m_elbowController.setSmartMotionMaxVelocity(600.0, 0);
    m_elbowController.setSmartMotionMaxAccel(450.0, 0);
    m_elbowController.setSmartMotionMinOutputVelocity(0.0, 0);
    m_elbowController.setSmartMotionAllowedClosedLoopError(1.0, 0);
    m_elbowMotor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    m_elbowMotor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);

    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderController = m_shoulderMotor.getPIDController();
    m_shoulderEncoder = m_shoulderMotor.getEncoder();    
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);  
    m_shoulderController.setP(ArmConstants.SHOULDER_PROPORTIONAL_GAIN);
    m_shoulderController.setSmartMotionMaxVelocity(600.0, 0);
    m_shoulderController.setSmartMotionMaxAccel(450.0, 0);
    m_shoulderController.setSmartMotionMinOutputVelocity(0.0, 0);
    m_shoulderController.setSmartMotionAllowedClosedLoopError(1.0, 0);
    m_shoulderMotor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    m_shoulderMotor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);

    /*shoulderConfig.forwardSoftLimitEnable = true;
    shoulderConfig.forwardSoftLimitThreshold = 0;
    shoulderConfig.reverseSoftLimitEnable = true;
    shoulderConfig.reverseSoftLimitThreshold = ArmConstants.SHOULDER_ONLY_HORIZONTAL_TICKS * 2.0;*/
  }

  public void resetPosition() {
    System.out.println("reset");
    m_shoulderMotor.getEncoder().setPosition(0.0);
    m_elbowMotor.getEncoder().setPosition(0.0);
  }

  public void setArmSpeed(double shoulderSpeed, double elbowSpeed) {
    m_shoulderMotor.set(shoulderSpeed);
    m_elbowMotor.set(elbowSpeed);
  }

  /**
   * @param placement the desired updated arm placement.
   */
  public void setKnownArmPlacement(KnownArmPlacement placement) {
    double desiredShoulderAngle = placement.m_shoulderAngle;
    double desiredShoulderTicks = desiredShoulderAngle * ArmConstants.SHOULDER_ROTATIONS_PER_DEGREE + ArmConstants.SHOULDER_HORIZONTAL_ROTATIONS;
    double desiredElbowAngle = placement.m_elbowAngle;
    if (!m_prototypeArm){
      desiredElbowAngle = desiredElbowAngle - (placement.m_shoulderAngle - 90.0);
    }
    double desiredElbowTicks = desiredElbowAngle * ArmConstants.ELBOW_ROTATIONS_PER_DEGREE
        + ArmConstants.ELBOW_HORIZONTAL_ROTATIONS;
    setShoulderPosition(desiredShoulderTicks);    
    setElbowPosition(desiredElbowTicks);
  }

  /**
   * @param targetPosition the target position in encoder ticks.
   */
  void setShoulderPosition(double targetPosition) {
    m_targetShoulderPosition = targetPosition / 42.0;
  }

  /**
   * @param targetPosition the target position in encoder ticks.
   */
  void setElbowPosition(double targetPosition) {
    m_targetElbowPosition = targetPosition / 42.0;
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
    double currentPos = m_elbowEncoder.getPosition();
    double degrees = (currentPos - ArmConstants.ELBOW_HORIZONTAL_ROTATIONS) / ArmConstants.ELBOW_ROTATIONS_PER_DEGREE;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_elbowController.setReference(
      m_targetElbowPosition, ControlType.kSmartMotion, 0, 
      ArmConstants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  void proceedToShoulderPosition() {
    double currentPos = m_shoulderEncoder.getPosition();
    double degrees = (currentPos - ArmConstants.SHOULDER_HORIZONTAL_ROTATIONS) / ArmConstants.SHOULDER_ROTATIONS_PER_DEGREE;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_elbowController.setReference(
      m_targetShoulderPosition, ControlType.kSmartMotion, 0, 
      ArmConstants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow rotations", m_elbowEncoder.getPosition());
    SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elbow current", m_elbowMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder rotations", m_shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder voltage", m_shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder current", m_shoulderMotor.getOutputCurrent());
  }
}
