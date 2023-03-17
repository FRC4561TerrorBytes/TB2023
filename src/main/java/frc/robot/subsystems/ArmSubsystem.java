package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameState;

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
    STOWED(101.0, -58.0),
    FLOOR_GRAB(65.0, -64.5),
    SUBSTATION_APPROACH(120.0, 7.2),
    SUBSTATION_GRAB_HALFWAY(108.0, 3.0),
    SUBSTATION_GRAB_FULLWAY(91.4, 0.9),
    SCORE_PREP_INITIAL(102.8, -57.2),
    SCORE_LOW(90.0, -53.0),
    SCORE_MIDDLE(90, -1.0),
    SCORE_CUBE_HIGH(56.0, 21.0),
    SCORE_CONE_HIGH(53.0, 30.0),
    SCORE_CONE_MIDDLE_UPPER(45.0, 35.0),
    SCORE_CONE_MIDDLE_LOWER(75.0, -15.0);

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
    m_elbowMotor.setInverted(false);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_elbowController.setP(Constants.ELBOW_PROPORTIONAL_GAIN_SLOT_0, 0);
    m_elbowController.setP(Constants.ELBOW_PROPORTIONAL_GAIN_SLOT_1, 1);
    m_elbowController.setD(Constants.ELBOW_DERIVATIVE_GAIN, 0);
    m_elbowController.setI(Constants.ELBOW_INTEGRAL_GAIN, 0);
    m_elbowController.setIZone(Constants.ELBOW_IZONE, 0);
    m_elbowController.setIMaxAccum(0.1, 0);
    m_elbowReverseLimitSwitch = m_elbowMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_elbowReverseLimitSwitch.enableLimitSwitch(true);
    m_elbowEncoder.setPositionConversionFactor(1.0 / Constants.ELBOW_ROTATIONS_PER_DEGREE);
    m_elbowMotor.setSmartCurrentLimit(60);
    m_elbowMotor.enableVoltageCompensation(12.0);
    m_elbowController.setOutputRange(-0.15, 0.25);
    //m_elbowMotor.setClosedLoopRampRate(1.0);
    //m_elbowMotor.setOpenLoopRampRate(1.0);
    
    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderController = m_shoulderMotor.getPIDController();
    m_shoulderEncoder = m_shoulderMotor.getEncoder();
    m_shoulderMotor.setInverted(true);
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderController.setP(Constants.SHOULDER_PROPORTIONAL_GAIN_SLOT_0, 0);
    m_shoulderController.setP(Constants.SHOULDER_PROPORTIONAL_GAIN_SLOT_1, 1);
    m_shoulderController.setD(Constants.SHOULDER_DERIVATIVE_GAIN);
    m_shoulderForwardLimitSwitch = m_shoulderMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_shoulderReverseLimitSwitch = m_shoulderMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_shoulderForwardLimitSwitch.enableLimitSwitch(true);
    m_shoulderReverseLimitSwitch.enableLimitSwitch(true);
    m_shoulderEncoder.setPositionConversionFactor(1.0 / Constants.SHOULDER_ROTATIONS_PER_DEGREE);
    m_shoulderMotor.setSmartCurrentLimit(30);
    m_shoulderMotor.enableVoltageCompensation(12.0);
    m_shoulderMotor.setClosedLoopRampRate(0.3);
    m_shoulderMotor.setOpenLoopRampRate(0.3);

    // NOTE: these are NEEDED
    resetShoulderPosition();
    resetElbowPosition();
  }

  public void resetShoulderPosition() {
    setShoulderPosition(Constants.SHOULDER_ZERO_OFFSET);
    m_shoulderEncoder.setPosition(Constants.SHOULDER_ZERO_OFFSET);
  }

  public void resetElbowPosition() {
    setElbowPosition(Constants.ELBOW_ZERO_OFFSET);
    m_elbowEncoder.setPosition(Constants.ELBOW_ZERO_OFFSET);
  }

  public boolean shoulderLimitReached() {
    return m_shoulderForwardLimitSwitch.isPressed();
  }

  public boolean elbowLimitReached() {
    return m_elbowReverseLimitSwitch.isPressed();
  }

  public void setManualArmSpeed(double shoulderSpeed, double elbowSpeed) {
    setManualShoulderSpeed(shoulderSpeed);
    setManualElbowSpeed(elbowSpeed);
  }

  public void setManualShoulderSpeed(double shoulderSpeed) {
    m_shoulderMotor.set(shoulderSpeed);
  }

  public void setManualElbowSpeed(double elbowSpeed) {
    m_elbowMotor.set(elbowSpeed);
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
   * @return current arm placement
   */
  public KnownArmPlacement getArmPlacement() {
    return m_lastPlacement;
  }

  public void nudgeShoulderForward() {
    setShoulderPosition(m_targetShoulderPosition - Constants.SHOULDER_NUDGE_DEGREES);
  }

  public void nudgeShoulderBackward() {
    setShoulderPosition(m_targetShoulderPosition + Constants.SHOULDER_NUDGE_DEGREES);
  }

  public void nudgeElbowUp() {
    setElbowPosition(m_targetElbowPosition + Constants.ELBOW_NUDGE_DEGREES);
  }

  public void nudgeElbowDown() {
    setElbowPosition(m_targetElbowPosition - 2.0);
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
    m_elbowController.setIAccum(0.0);
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
    double cosineScalar = Math.cos(Math.toRadians(currentRotation + m_shoulderEncoder.getPosition() - 90.0));

    int pidSlot = 0;
    if (currentRotation > m_targetElbowPosition) {
      pidSlot = 1;
      cosineScalar = 0;
    }

    m_elbowController.setReference(
        m_targetElbowPosition, ControlType.kPosition, pidSlot,
        Constants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  void proceedToShoulderPosition() {
    double currentRotation = m_shoulderEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentRotation));

    int pidSlot = 0;
    if (currentRotation > m_targetShoulderPosition) {
      pidSlot = 1;
    }

    m_shoulderController.setReference(
        m_targetShoulderPosition, ControlType.kPosition, pidSlot,
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
    SmartDashboard.putNumber("Elbow rotations to ground",
        m_elbowEncoder.getPosition() + m_shoulderEncoder.getPosition() - 90.0);
    SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getBusVoltage() * m_elbowMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elbow current", m_elbowMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elbow rotation target", m_targetElbowPosition);
    SmartDashboard.putNumber("Elbow placement", m_lastPlacement == null ? 999 :
      m_lastPlacement.m_elbowAngle);
    SmartDashboard.putBoolean("Elbow LimitR", m_elbowReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoudler LimitR", m_shoulderReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoulder LimitF", m_shoulderForwardLimitSwitch.isPressed());
    SmartDashboard.putNumber("Shoulder rotations", m_shoulderEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder voltage", m_shoulderMotor.getBusVoltage() * m_shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder current", m_shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder rotation target", m_targetShoulderPosition);
    SmartDashboard.putBoolean("Game Piece Held", GameState.getInstance().isGamePieceHeld());
    SmartDashboard.putNumber("Elbow Temp (C)", m_elbowMotor.getMotorTemperature());
    // SmartDashboard.putNumber("Shoulder placement", m_lastPlacement == null ? 999
    // : m_lastPlacement.m_shoulderAngle);
    /*
     * double elbowKP = SmartDashboard.getNumber("Elbow KP", 0);
     * double shoulderKP = SmartDashboard.getNumber("Elbow KP", 0);
     * m_elbowController.setP(elbowKP);
     * m_shoulderController.setP(shoulderKP);
     */
  }
}
