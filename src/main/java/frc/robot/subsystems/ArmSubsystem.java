package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_shoulderEncoder;
  private SparkMaxPIDController m_shoulderController;
  private SparkMaxLimitSwitch m_shoulderForwardLimitSwitch;
  private SparkMaxLimitSwitch m_shoulderReverseLimitSwitch;
  // private RelativeEncoder m_shoulderThrougboreEncoder;
  private SparkMaxAbsoluteEncoder m_shoulderThroughboreEncoder;
  private CANSparkMax m_elbowMotor = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_elbowEncoder;
  private SparkMaxPIDController m_elbowController;
  private SparkMaxLimitSwitch m_elbowReverseLimitSwitch;
  private SparkMaxAbsoluteEncoder m_elbowThroughboreEncoder;
  private CANSparkMax m_wristMotor = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
  private RelativeEncoder m_wristEncoder;
  private SparkMaxAbsoluteEncoder m_wristThrougboreEncoder;
  private SparkMaxPIDController m_wristController;

  private KnownArmPlacement m_lastPlacement = null;
  private double m_targetShoulderPosition = 0.0;
  private double m_targetElbowPosition = 0.0;
  private double m_targetWristPosition = 0.0;

  private TrapezoidProfile m_shoulderMotionProfile;
  private TrapezoidProfile m_elbowMotionProfile;

  /**
   * An enumeration of known arm placements, e.g. stowed or score cone high). The
   * angles are in degrees from horizontal when the controlled arm segment is
   * pointing forward. We use horizontal as 0 for our degree measurements rather
   * than vertical due to how cosine and standard trigonometry angle measurements
   * work.
   */
  public enum KnownArmPlacement {
    STOWED(101.0, -58.0, 0.0),
    FLOOR_GRAB_CONE(86.0, -74.0, 65.0), // TODO: Change to intake knocked over cones
    FLOOR_GRAB_CUBE(54.0, -80.0, 33.0),
    FLOOR_GRAB_PRE(54.0, -80.0, 0.0),
    SUBSTATION_APPROACH(92.5, 11.5, 50.0),
    SUBSTATION_GRAB_HALFWAY_CUBE(108.0, 3.0, 85.0),
    SUBSTATION_GRAB_HALFWAY_CONE(108.0, 23.0, 161.0),
    SUBSTATION_GRAB_FULLWAY_CUBE(91.4, 0.9, 60.0),
    SUBSTATION_GRAB_FULLWAY_CONE(91.4, 0.9, 150.0),
    SINGLE_SUBSTATION_CUBE(101.0, -45.0, 0.0),
    SINGLE_SUBSTATION_CONE(91.4, -66.5, 14.0),
    SCORE_LOW_CUBE(90.0, -53.0, 0.0),
    SCORE_LOW_CONE(93.0, -68.0, 72.0),
    SCORE_MIDDLE_CUBE(91.0, -1.0, 73.0),
    // SCORE_CONE_ MIDDLE_UPPER(63.0, 35.0, 150.0),
    SCORE_CONE_MIDDLE(92.5, 11.5, 162.0),
    SCORE_CUBE_HIGH(68.0, 20.0, 68.0),
    SCORE_CONE_HIGH_PRE(55.0, 40.0, 0.0),
    SCORE_CONE_HIGH(64.0, 36.0, 70.0),
    SCORE_CONE_HIGH_WRIST(61.0, 33.0, 162.0),
    SCORE_CONE_HIGH_RETURN(92.5, 50.0, 0.0);

    public final double m_shoulderAngle;
    public final double m_elbowAngle;
    public final double m_wristAngle;

    private KnownArmPlacement(double shoulderAngle, double elbowAngle, double wristAngle) {
      m_shoulderAngle = shoulderAngle;
      m_elbowAngle = elbowAngle;
      m_wristAngle = wristAngle;
    }
  }

  double elbowMaxVelocity = 75.0; // degrees per second
  double elbowMaxAcceleration = 360; //degrees per second^2

  double shoulderMaxVelocity = 150.0;// degrees per second
  double shoulderMaxAcceleration = 240.0;//degrees per second^2

  private TrapezoidProfile.Constraints shoulderConstraints = new TrapezoidProfile.Constraints(shoulderMaxVelocity,
      shoulderMaxAcceleration);
  private TrapezoidProfile.Constraints elbowConstraints = new TrapezoidProfile.Constraints(elbowMaxVelocity,
      elbowMaxAcceleration);

  private Timer m_shoulderTimer = new Timer();
  private Timer m_elbowTimer = new Timer();

  public ArmSubsystem() {
    m_elbowMotor.restoreFactoryDefaults();
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setSmartCurrentLimit(60);
    m_elbowMotor.enableVoltageCompensation(12.0);
    m_elbowMotor.setInverted(false);
    m_elbowEncoder = m_elbowMotor.getEncoder();
    m_elbowEncoder.setPositionConversionFactor(1.0 / Constants.ELBOW_ROTATIONS_PER_DEGREE);
    m_elbowController = m_elbowMotor.getPIDController();
    m_elbowController.setOutputRange(-0.15, 0.25);
    m_elbowController.setP(Constants.ELBOW_PROPORTIONAL_GAIN_SLOT_0, 0);
    m_elbowController.setP(Constants.ELBOW_PROPORTIONAL_GAIN_SLOT_1, 1);
    m_elbowController.setD(Constants.ELBOW_DERIVATIVE_GAIN_SLOT_0, 0);
    m_elbowController.setD(Constants.ELBOW_DERIVATIVE_GAIN_SLOT_1, 1);
    m_elbowController.setI(Constants.ELBOW_INTEGRAL_GAIN, 0);
    m_elbowController.setIZone(Constants.ELBOW_IZONE, 0);
    m_elbowController.setIMaxAccum(0.1, 0);
    m_elbowReverseLimitSwitch = m_elbowMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_elbowReverseLimitSwitch.enableLimitSwitch(true);
    // m_elbowMotor.setClosedLoopRampRate(1.0);
    // m_elbowMotor.setOpenLoopRampRate(1.0);
    m_elbowThroughboreEncoder = m_elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_elbowThroughboreEncoder.setInverted(true);
    m_elbowThroughboreEncoder.setPositionConversionFactor(360.0);
    m_elbowThroughboreEncoder.setZeroOffset(220.0);

    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderMotor.setSmartCurrentLimit(30);
    m_shoulderMotor.enableVoltageCompensation(12.0);
    m_shoulderMotor.setClosedLoopRampRate(0.3);
    m_shoulderMotor.setOpenLoopRampRate(0.3);
    m_shoulderMotor.setInverted(true);
    m_shoulderEncoder = m_shoulderMotor.getEncoder();
    m_shoulderEncoder.setPositionConversionFactor(1.0 / Constants.SHOULDER_ROTATIONS_PER_DEGREE);
    m_shoulderController = m_shoulderMotor.getPIDController();
    m_shoulderController.setP(Constants.SHOULDER_PROPORTIONAL_GAIN_SLOT_0, 0);
    m_shoulderController.setP(Constants.SHOULDER_PROPORTIONAL_GAIN_SLOT_1, 1);
    m_shoulderController.setI(Constants.SHOULDER_INTEGRAL_GAIN_SLOT_0, 0);
    m_shoulderController.setD(Constants.SHOULDER_DERIVATIVE_GAIN);
    m_shoulderForwardLimitSwitch = m_shoulderMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_shoulderReverseLimitSwitch = m_shoulderMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    m_shoulderForwardLimitSwitch.enableLimitSwitch(true);
    m_shoulderReverseLimitSwitch.enableLimitSwitch(true);
    m_shoulderThroughboreEncoder = m_shoulderMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_shoulderThroughboreEncoder.setInverted(true);
    m_shoulderThroughboreEncoder.setPositionConversionFactor(360.0);
    m_shoulderThroughboreEncoder.setZeroOffset(0.0);
    // m_shoulderController.setFeedbackDevice(m_shoulderThroughboreEncoder);

    m_wristMotor.restoreFactoryDefaults();
    m_wristController = m_wristMotor.getPIDController();
    m_wristEncoder = m_wristMotor.getEncoder();
    m_wristMotor.setInverted(false);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristController.setP(Constants.WRIST_PROPORTIONAL_GAIN);
    m_wristMotor.setClosedLoopRampRate(0.5);
    m_wristMotor.setClosedLoopRampRate(0.5);
    m_wristThrougboreEncoder = m_wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_wristThrougboreEncoder.setPositionConversionFactor(360.0);
    m_wristThrougboreEncoder.setInverted(false);
    m_wristThrougboreEncoder.setZeroOffset(95.0);
    m_wristMotor.setSmartCurrentLimit(20);
    m_wristEncoder.setPositionConversionFactor(1.0 / Constants.WRIST_ROTATIONS_PER_DEGREE);
    // m_wristMotor.setSoftLimit(null, 0)
    // m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, 160f);
    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, 0f);
    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    seedRelativeEncoders();
    setTargetsToCurrents();

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

  public void setManualWristSpeed(double wristSpeed) {
    m_wristMotor.set(wristSpeed);
  }

  /**
   * @param placement the desired updated arm placement.
   */
  public void setKnownArmPlacement(final KnownArmPlacement placement) {
    double shoulderRotation = m_shoulderEncoder.getPosition();
    double elbowRotation = m_elbowEncoder.getPosition();
    double desiredShoulderAngle = placement.m_shoulderAngle + Constants.SHOULDER_POSITION_OFFSET;
    double desiredElbowAngle = placement.m_elbowAngle + Constants.ELBOW_POSITION_OFFSET - placement.m_shoulderAngle
        + 90;
    double desiredWristAngle = placement.m_wristAngle;

    TrapezoidProfile.State desiredShoulderState = new TrapezoidProfile.State(desiredShoulderAngle, 0.0);
    TrapezoidProfile.State currentShoulderState = new TrapezoidProfile.State(shoulderRotation,
        m_shoulderEncoder.getVelocity() / 60);
    TrapezoidProfile.State desiredElbowState = new TrapezoidProfile.State(desiredElbowAngle, 0.0);
    TrapezoidProfile.State currentElbowState = new TrapezoidProfile.State(elbowRotation,
        m_elbowEncoder.getVelocity() / 60);

    m_shoulderMotionProfile = new TrapezoidProfile(shoulderConstraints, desiredShoulderState, currentShoulderState);
    m_elbowMotionProfile = new TrapezoidProfile(elbowConstraints, desiredElbowState, currentElbowState);

    m_elbowTimer.restart();
    m_shoulderTimer.restart();

    setWristPosition(desiredWristAngle);
    if (shoulderRotation < desiredShoulderAngle) {
      setShoulderPosition(desiredShoulderAngle);
      new WaitCommand(0.5)
          .andThen(new InstantCommand(() -> setElbowPosition(desiredElbowAngle))).schedule();
    } else {
      setElbowPosition(desiredElbowAngle);
      new WaitCommand(0.25)
          .andThen(new InstantCommand(() -> setShoulderPosition(desiredShoulderAngle))).schedule();
    }
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

  public void nudgeWristUp() {
    setWristPosition(m_targetWristPosition - Constants.WRIST_NUDGE_DEGREES);
  }

  public void nudgeWristDown() {
    setWristPosition(m_targetWristPosition + Constants.WRIST_NUDGE_DEGREES);
  }

  /**
   * @param targetPosition the target position in rotations.
   */
  void setShoulderPosition(double targetPosition) {
    m_shoulderController.setIAccum(0.0);
    m_targetShoulderPosition = targetPosition;
  }

  /**
   * @param targetPosition the target position in rotations.
   */
  void setElbowPosition(double targetPosition) {
    m_elbowController.setIAccum(0.0);
    m_targetElbowPosition = targetPosition;
  }

  /**
   * @param targetPosition the target position in rotations.
   */
  void setWristPosition(double targetPosition) {
    m_targetWristPosition = targetPosition;
  }

  public void proceedToArmPosition() {
    proceedToShoulderPosition();
    proceedToElbowPosition();
    proceedToWristPosition();
  }

  /**
   * @return Elbow Position in Degrees
   */
  private double getCalculatedElbowAngle() {
    double elbowAngle = m_elbowThroughboreEncoder.getPosition();

    if (elbowAngle > 180) { // Safety factor to invert sign of elbow angle in position that elbow can't
                            // reach
      return elbowAngle - 360;
    } else {
      return elbowAngle;
    }
  }

  /**
   * @return Shoulder Position in Degrees
   */
  private double getCalculatedShoulderAngle() {
    return (m_shoulderThroughboreEncoder.getPosition() + 25) / Constants.SHOULDER_ENCODER_ROTATIONS_PER_DEGREE;
  }

  /**
   * @return Wrist Position in Degrees
   */
  private double getCalculatedWristAngle() {
    return -(m_wristThrougboreEncoder.getPosition() + Constants.WRIST_ENCODER_OFFSET) % 360;
  }

  /**
   * Scales the relative encoder positions to match the degrees read from absolute
   * encoders
   * (Shoulder, Elbow, and Wrist)
   */
  public void seedRelativeEncoders() {
    m_elbowEncoder.setPosition(getCalculatedElbowAngle());
    m_shoulderEncoder.setPosition(getCalculatedShoulderAngle());
    m_wristEncoder.setPosition(m_wristThrougboreEncoder.getPosition());
  }

  /**
   * Drives the elbow toward the last postion set via
   * {@link} {@link #setElbowPosition(double)}}.
   */
  void proceedToElbowPosition() {
    double currentDegrees = m_elbowEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentDegrees + m_shoulderEncoder.getPosition() - 90.0));

    int pidSlot = 0;
    if (currentDegrees > m_targetElbowPosition) {
      pidSlot = 1;
      cosineScalar = 0;
    }
    if (m_elbowMotionProfile != null) {
      if (m_elbowMotionProfile.isFinished(m_elbowTimer.get())) {
        m_elbowController.setReference(
          m_targetElbowPosition, ControlType.kPosition, pidSlot,
          Constants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
      } else {
        m_elbowController.setReference(
          m_elbowMotionProfile.calculate(m_elbowTimer.get()).position, ControlType.kPosition, pidSlot,
          Constants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
      }
    } else {
      m_elbowController.setReference(
        m_targetElbowPosition, ControlType.kPosition, pidSlot,
        Constants.ELBOW_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
    }
  }

  void proceedToShoulderPosition() {
    double currentDegrees = m_shoulderEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentDegrees));

    int pidSlot = 0;
    if (currentDegrees < m_targetShoulderPosition) {
      pidSlot = 1;
    }
    if (m_shoulderMotionProfile != null) {
      if (m_shoulderMotionProfile.isFinished(m_shoulderTimer.get())) {
        m_shoulderController.setReference(
            m_targetShoulderPosition, ControlType.kPosition, pidSlot,
            Constants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
      } else {
        m_shoulderController.setReference(
            m_shoulderMotionProfile.calculate(m_shoulderTimer.get()).position, ControlType.kPosition, pidSlot,
            Constants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
      }
    } else {
      m_shoulderController.setReference(
        m_targetShoulderPosition, ControlType.kPosition, pidSlot,
        Constants.SHOULDER_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
    }
  }

  void proceedToWristPosition() {
    double currentDegrees = m_wristEncoder.getPosition();
    double cosineScalar = Math.cos(Math.toRadians(currentDegrees));

    m_wristController.setReference(
        m_targetWristPosition, ControlType.kPosition, 0,
        Constants.WRIST_MAX_VOLTAGE_FF * cosineScalar, ArbFFUnits.kVoltage);
  }

  public void setTargetsToCurrents() {
    m_targetShoulderPosition = getCalculatedShoulderAngle();
    m_targetElbowPosition = getCalculatedElbowAngle();
    m_targetWristPosition = m_wristThrougboreEncoder.getPosition();
    m_lastPlacement = null;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elbow rotations", m_elbowEncoder.getPosition());
    // SmartDashboard.putNumber("Elbow rotations to ground",
    // m_elbowEncoder.getPosition() + m_shoulderEncoder.getPosition() - 90.0);
    // SmartDashboard.putNumber("Elbow voltage", m_elbowMotor.getBusVoltage() *
    // m_elbowMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Elbow current", m_elbowMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elbow rotation target", m_targetElbowPosition);
    // SmartDashboard.putNumber("Elbow placement", m_lastPlacement == null ? 999 :
    // m_lastPlacement.m_elbowAngle);
    SmartDashboard.putBoolean("Elbow LimitR",
        m_elbowReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoudler LimitR",
        m_shoulderReverseLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Shoulder LimitF",
        m_shoulderForwardLimitSwitch.isPressed());
    SmartDashboard.putNumber("Shoulder rotations", m_shoulderEncoder.getPosition());
    // SmartDashboard.putNumber("Shoulder voltage", m_shoulderMotor.getBusVoltage()
    // * m_shoulderMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Shoulder current",
    // m_shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder rotation target", m_targetShoulderPosition);
    SmartDashboard.putNumber("wrist rotations", m_wristEncoder.getPosition());
    // SmartDashboard.putNumber("wrist voltage", m_wristMotor.getBusVoltage() *
    // m_wristMotor.getAppliedOutput());
    // SmartDashboard.putNumber("wrist current", m_wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("wrist rotation target", m_targetWristPosition);
    // SmartDashboard.putBoolean("Game Piece Held",
    // GameState.getInstance().isGamePieceHeld());
    // SmartDashboard.putNumber("Elbow Temp (C)",
    // m_elbowMotor.getMotorTemperature());
    // SmartDashboard.putNumber("Wrist Velocity", m_wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Throughbore Pos", getCalculatedWristAngle());
    SmartDashboard.putNumber("Wrist Throubore Encoder", m_wristThrougboreEncoder.getPosition());
    SmartDashboard.putNumber(("Elbow Calculated Angle"), getCalculatedElbowAngle());
    SmartDashboard.putNumber(("Shoulder Throughbore Encoder"), m_shoulderThroughboreEncoder.getPosition());
    SmartDashboard.putNumber("Elbow Througbore Encoder", m_elbowThroughboreEncoder.getPosition());
    SmartDashboard.putNumber(("Shoulder Calculated Angle"), getCalculatedShoulderAngle());

    Logger.getInstance().recordOutput("Calculated Shoulder", getCalculatedShoulderAngle());
    Logger.getInstance().recordOutput("Calculated Elbow", getCalculatedElbowAngle());
    Logger.getInstance().recordOutput("Calculated Wrist", getCalculatedWristAngle());
  
    Logger.getInstance().recordOutput("Shoulder Target", m_targetShoulderPosition);
    Logger.getInstance().recordOutput("Elbow Target", m_targetElbowPosition);
    Logger.getInstance().recordOutput("Wrist Target", m_targetWristPosition);

  }
}
