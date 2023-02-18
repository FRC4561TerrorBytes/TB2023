package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
// private WPI_TalonFX m_elbowMotor = new WPI_TalonFX(4);
// private WPI_TalonFX m_shoulderMotor = new WPI_TalonFX(12);
  private CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushless);  
  private RelativeEncoder m_shoulderEncoder; 
  private SparkMaxPIDController m_shoulderController;
  private SparkMaxLimitSwitch m_shoulderForwardLimitSwitch;
  private SparkMaxLimitSwitch m_shoulderReverseLimitSwitch;
  private CANSparkMax m_elbowMotor = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_elbowEncoder;
  private SparkMaxPIDController m_elbowController;
  private SparkMaxLimitSwitch m_elbowReverseLimitSwitch;

  private KnownArmPlacement m_lastPlacement = null;
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
    STARTING(90.0, -85.0),
    SUBSTATION_APPROACH(125.0, 5.2),
    SUBSTATION_GRAB_HALFWAY(116.0, 1.0),
    SUBSTATION_GRAB_FULLWAY(107.4, -2.9),
    SCORE_PREP_INITIAL(102.8, -57.2),
    SCORE_LOW(90.0, -53.0),
    SCORE_MIDDLE(90.0, -7.0),
    SCORE_HIGH(56.0, 23.0);

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
    // m_elbowMotor.setInverted(false);  
    m_elbowMotor.setIdleMode(IdleMode.kBrake);  
    m_elbowController.setP(Constants.ELBOW_PROPORTIONAL_GAIN);
    m_elbowController.setSmartMotionMaxVelocity(Constants.ELBOW_CRUISE_VELOCITY_RPM, 0);
    m_elbowController.setSmartMotionMaxAccel(Constants.ELBOW_PEAK_ACCELERATION, 0);
    m_elbowController.setSmartMotionMinOutputVelocity(0.0, 0);
    m_elbowController.setSmartMotionAllowedClosedLoopError(Constants.ELBOW_TOLERANCE, 0);
    m_elbowReverseLimitSwitch = m_elbowMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_elbowReverseLimitSwitch.enableLimitSwitch(true);
    m_elbowEncoder.setPositionConversionFactor(1.0 / Constants.ELBOW_ROTATIONS_PER_DEGREE);
    m_elbowMotor.setSmartCurrentLimit(60);

    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderController = m_shoulderMotor.getPIDController();
    m_shoulderEncoder = m_shoulderMotor.getEncoder();    
    m_shoulderMotor.setInverted(true);
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);  
    m_shoulderController.setP(Constants.SHOULDER_PROPORTIONAL_GAIN);
    m_shoulderController.setSmartMotionMaxVelocity(Constants.SHOULDER_CRUISE_VELOCITY_RPM, 0);
    m_shoulderController.setSmartMotionMaxAccel(Constants.SHOULDER_PEAK_ACCELERATION, 0);
    m_shoulderController.setSmartMotionMinOutputVelocity(0.0, 0);
    m_shoulderController.setSmartMotionAllowedClosedLoopError(Constants.SHOULDER_TOLERANCE, 0);
    m_shoulderForwardLimitSwitch = m_shoulderMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_shoulderReverseLimitSwitch = m_shoulderMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_shoulderForwardLimitSwitch.enableLimitSwitch(true);
    m_shoulderReverseLimitSwitch.enableLimitSwitch(true);
    m_shoulderEncoder.setPositionConversionFactor(1.0 / Constants.SHOULDER_ROTATIONS_PER_DEGREE);
    m_shoulderMotor.setSmartCurrentLimit(30);
    
    //resetPosition();
  }

  public void resetShoulderPosition() {
    setShoulderPosition(Constants.SHOULDER_ZERO_OFFSET);
    m_shoulderEncoder.setPosition(Constants.SHOULDER_ZERO_OFFSET);
  }

  public void resetElbowPosition(){
    setElbowPosition(Constants.ELBOW_ZERO_OFFSET);
    m_elbowEncoder.setPosition(Constants.ELBOW_ZERO_OFFSET);
  }

  public boolean shoulderLimitReached() {
    return m_shoulderForwardLimitSwitch.isPressed();
  }

  public boolean elbowLimitReached(){
    return m_shoulderForwardLimitSwitch.isPressed();
  }

  public void setArmSpeed(double shoulderSpeed, double elbowSpeed) {
    m_shoulderMotor.set(shoulderSpeed);
    m_elbowMotor.set(elbowSpeed);
  }

  public void setArmDifferential(double shoulderSpeed, double elbowSpeed) {
    m_targetShoulderPosition += shoulderSpeed / 60;
    m_targetElbowPosition += (elbowSpeed - shoulderSpeed) / 60;
  }

  /**
   * @param placement the desired updated arm placement.
   */
  public void setKnownArmPlacement(final KnownArmPlacement placement) {
    double desiredShoulderAngle = placement.m_shoulderAngle;
    double desiredElbowAngle = placement.m_elbowAngle - placement.m_shoulderAngle + 90;
    setShoulderPosition(desiredShoulderAngle);    
    setElbowPosition(desiredElbowAngle);
    m_lastPlacement = placement;
  }

  /**
   * @param targetPosition the target position in rotations.
   */
  void setShoulderPosition(double targetPosition) {
    m_targetShoulderPosition = targetPosition;
  }

  /**
   * @param targetPosition the target position in rotations.
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
    double currentRotation = m_elbowEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentRotation));
    m_elbowController.setReference(
      m_targetElbowPosition, ControlType.kSmartMotion, 0, 
      Constants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  void proceedToShoulderPosition() {
    double currentRotation = m_shoulderEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentRotation));
    m_shoulderController.setReference(
      m_targetShoulderPosition, ControlType.kSmartMotion, 0, 
      Constants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  public void setTargetsToCurrents() {
    m_targetShoulderPosition = m_shoulderEncoder.getPosition();
    m_targetElbowPosition = m_elbowEncoder.getPosition();
    m_lastPlacement = null;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow rotations", m_elbowEncoder.getPosition());
    SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getBusVoltage() * m_elbowMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elbow current", m_elbowMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elbow rotation target", m_targetElbowPosition);
    SmartDashboard.putNumber("Elbow placement", m_lastPlacement == null ? 999 : m_lastPlacement.m_elbowAngle);
    SmartDashboard.putBoolean("Elbow LimitR", m_elbowReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoudler LimitR", m_shoulderReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoulder LimitF", m_shoulderForwardLimitSwitch.isPressed());
    SmartDashboard.putNumber("Shoulder rotations", m_shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder voltage", m_shoulderMotor.getBusVoltage() * m_shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder current", m_shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder rotation target", m_targetShoulderPosition);
    SmartDashboard.putNumber("Shoulder placement", m_lastPlacement == null ? 999 : m_lastPlacement.m_shoulderAngle);
    /*double elbowKP = SmartDashboard.getNumber("Elbow KP", 0);
    double shoulderKP = SmartDashboard.getNumber("Elbow KP", 0);
    m_elbowController.setP(elbowKP);
    m_shoulderController.setP(shoulderKP);*/
  }
}
