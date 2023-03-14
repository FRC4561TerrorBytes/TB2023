package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SparkPIDConfig;

public class SwerveModule extends SubsystemBase {

  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANCoder m_absEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final double m_steerOffset;
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
      SparkPIDConfig turnMotorConfig, boolean driveInverted, boolean turnInverted) {

    m_driveMotor = new TalonFX(driveMotorPort);
    m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
    m_absEncoder = new CANCoder(absEncoderID);
    m_turnEncoder = m_turnMotor.getEncoder();
    m_steerOffset = steerOffset;
    m_driveMotorCurrentLimit = new StatorCurrentLimitConfiguration(
        true,
        Constants.DRIVE_CURRENT_LIMIT,
        Constants.DRIVE_CURRENT_THRESHOLD,
        Constants.DRIVE_CURRENT_TIME_THRESHOLD);

    var turnPidController = turnMotorConfig.initializeSparkPID(m_turnMotor);

    turnPidController.setPositionPIDWrappingEnabled(true);
    turnPidController.setPositionPIDWrappingMaxInput(+Math.PI);
    turnPidController.setPositionPIDWrappingMinInput(-Math.PI);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turnMotor.setIdleMode(IdleMode.kCoast);

    m_driveMotor.setInverted(driveInverted);
    m_turnMotor.setInverted(turnInverted);

    m_driveMotor.configStatorCurrentLimit(m_driveMotorCurrentLimit);
    m_turnMotor.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);

    // Seed the relative encoders with absolute values after a couple seconds to
    // ensure correct values
    // (new WaitCommand(5)).andThen(new PrintCommand("balls"), new InstantCommand(()
    // -> m_turnEncoder.setPosition(m_absEncoder.getAbsolutePosition() -
    // steerOffset))).schedule();
    // m_turnEncoder.setPosition(m_absEncoder.getAbsolutePosition() - steerOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        10 * m_driveMotor.getSelectedSensorVelocity() * Constants.DRIVE_MOTOR_CONVERSION_FACTOR,
        new Rotation2d(m_turnEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d((m_turnEncoder.getPosition())));

    m_driveMotor.set(TalonFXControlMode.PercentOutput,
        state.speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND);
    m_turnMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition() * Constants.DRIVE_MOTOR_CONVERSION_FACTOR,
        new Rotation2d(m_turnEncoder.getPosition()));
  }

  public String toString() {
    String res = "\n\tABS-POS: ";
    res += m_absEncoder.getAbsolutePosition() % 180;
    res += "\n\tREL-POS: ";
    res += Math.toDegrees(m_turnEncoder.getPosition()) % 180;
    return res;
  }

  int resetLoop = 0;

  @Override
  public void periodic() {
    if (resetLoop++ == 300) {
      m_turnEncoder.setPosition(Math.toRadians(m_absEncoder.getAbsolutePosition() - m_steerOffset));
    }
  }
}
