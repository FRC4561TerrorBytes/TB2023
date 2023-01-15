package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANCoder m_absEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final StatorCurrentLimitConfiguration m_driveMotorCurrentLimit;

  // Gains are for example purposes only - must be determined for your own robot!
  // private final SimpleMotorFeedforward m_driveFeedforward = new
  // SimpleMotorFeedforward(1, 3);
  // private final SimpleMotorFeedforward m_turnFeedforward = new
  // SimpleMotorFeedforward(1, 0.5);

  /**
   * Creates a swerve module with a drive motor,
   * turn motor, absolute encoder, and steer offset.
   * 
   * @param driveMotorPort CAN ID for Falcon Drive Motor
   * @param turnMotorPort  CAN ID for NEO Turn Motor
   * @param absEncoderID   CAN ID for CANCoder Absolute Encoder per module
   * @param steerOffset    Encoder steer offset for each module (in radians)
   */
  public SwerveModule(int driveMotorPort, int turnMotorPort, int absEncoderID, double steerOffset,
      TalonPIDConfig driveMotorConfig, SparkPIDConfig turnMotorConfig) {

    m_driveMotor = new TalonFX(driveMotorPort);
    m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
    m_absEncoder = new CANCoder(absEncoderID);
    m_turnEncoder = m_turnMotor.getEncoder();
    m_driveMotorCurrentLimit = new StatorCurrentLimitConfiguration(
        true,
        Constants.DRIVE_CURRENT_LIMIT,
        Constants.DRIVE_CURRENT_THRESHOLD,
        Constants.DRIVE_CURRENT_TIME_THRESHOLD);

    driveMotorConfig.initializeTalonPID(m_driveMotor, FeedbackDevice.IntegratedSensor);
    turnMotorConfig.initializeSparkPID(m_turnMotor);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turnMotor.setIdleMode(IdleMode.kBrake);

    m_driveMotor.configStatorCurrentLimit(m_driveMotorCurrentLimit);
    m_turnMotor.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turnEncoder.getPosition()));
    // TODO: Convert to meters per sec, convert to radians
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d((m_turnEncoder.getPosition())));

    // final double turnFeedforward =
    // m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond);
    m_turnMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition( // TODO: convert from sensor units to meters
        m_driveMotor.getSelectedSensorPosition(), new Rotation2d(m_turnEncoder.getPosition()));
  }
}
