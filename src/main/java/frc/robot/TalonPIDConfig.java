/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.math.MathUtil;

/**
 * Automates the configuration of Talon PID and MotionMagic parameters
 */
public class TalonPIDConfig {
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MOTOR_DEADBAND = 0.01;
  private static final double MIN_TOLERANCE = 1.0;
  private static final int MIN_MOTION_SMOOTHING = 0;
  private static final int MAX_MOTION_SMOOTHING = 8;
  private static final int PID_SLOT = 0;

  private boolean m_motionMagic = false;
  private boolean m_enableSoftLimits = true;

  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_ticksPerRotation = 0.0;
  private double m_maxRPM = 0.0;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;

  private double m_velocityRPM = 1.0;
  private double m_accelerationRPMPerSec = 1.0;
  private int m_motionSmoothing = 0;

  /**
   * Create a TalonPIDConfig, without MotionMagic parameters
   * <p>
   * USE FOR VELOCITY PID ONLY!
   * 
   * @param sensorPhase          set sensor phase of encoder
   * @param invertMotor          invert motor or not
   * @param maxRPM               max RPM of encoder
   * @param ticksPerRotation     number of ticks in one encoder revolution
   * @param kP                   proportional gain
   * @param kI                   integral gain
   * @param kD                   derivative gain
   * @param mechanicalEfficiency mechanical efficiency of mechanism [0.0, +1.0]
   * @param tolerance            tolerance of PID loop in ticks per 100ms
   */
  public TalonPIDConfig(boolean sensorPhase, boolean invertMotor,
      double maxRPM, double ticksPerRotation,
      double kP, double kI, double kD,
      double mechanicalEfficiency, double tolerance) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_maxRPM = maxRPM * mechanicalEfficiency;
    this.m_ticksPerRotation = ticksPerRotation;
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);

    this.m_enableSoftLimits = false;
  }

  /**
   * Create a TalonPIDConfig, with MotionMagic parameters
   * <p>
   * USE FOR POSITION PID ONLY!
   * 
   * @param sensorPhase           set sensor phase of encoder
   * @param invertMotor           invert motor or not
   * @param ticksPerRotation      number of ticks in one encoder revolution
   * @param maxRPM                max RPM for this motor
   * @param kP                    proportional gain
   * @param kI                    integral gain
   * @param kD                    derivative gain
   * @param mechanicalEfficiency  mechanical efficiency of mechanism [0.0, +1.0]
   * @param tolerance             tolerance of PID loop in ticks
   * @param velocity              MotionMagic cruise velocity in RPM
   * @param accelerationRPMPerSec MotionMagic acceleration in RPM
   * @param motionSmoothing       MotionMagic smoothing factor [0, 8]
   */
  public TalonPIDConfig(boolean sensorPhase, boolean invertMotor, double ticksPerRotation, double maxRPM,
      double kP, double kI, double kD, double mechanicalEfficiency, double tolerance,
      double lowerLimit, double upperLimit, boolean enableSoftLimits,
      double velocityRPM, double accelerationRPMPerSec, int motionSmoothing) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_ticksPerRotation = ticksPerRotation;
    this.m_maxRPM = maxRPM * MathUtil.clamp(mechanicalEfficiency, 0.0, 1.0);
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_enableSoftLimits = enableSoftLimits;

    this.m_velocityRPM = velocityRPM;
    this.m_accelerationRPMPerSec = accelerationRPMPerSec;
    this.m_motionSmoothing = MathUtil.clamp(motionSmoothing, MIN_MOTION_SMOOTHING, MAX_MOTION_SMOOTHING);

    this.m_motionMagic = true;
  }

  /**
   * Initializes Talon PID and MotionMagic parameters
   * 
   * @param talon              Talon motor controller to apply settings to
   * @param feedbackDevice     Feedback device to use for Talon PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeTalonPID(BaseTalon talon, FeedbackDevice feedbackDevice,
      boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Reset Talon to default
    talon.configFactoryDefault();

    // Configure feedback sensor
    talon.configSelectedFeedbackSensor(feedbackDevice);

    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      talon.configForwardSoftLimitThreshold(m_upperLimit);
      talon.configForwardSoftLimitEnable(true);
      talon.configReverseSoftLimitThreshold(m_lowerLimit);
      talon.configReverseSoftLimitEnable(true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) {
      talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }
    if (reverseLimitSwitch) {
      talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    // Set sensor phase and invert motor if required
    talon.setSensorPhase(m_sensorPhase);
    talon.setInverted(m_invertMotor);

    // Configure PID values
    talon.config_kP(PID_SLOT, m_kP);
    talon.config_kI(PID_SLOT, m_kI);
    talon.config_kD(PID_SLOT, m_kD);
    talon.configAllowableClosedloopError(PID_SLOT, m_tolerance);
    talon.configClosedLoopPeakOutput(PID_SLOT, 1.0);

    talon.config_IntegralZone(PID_SLOT, m_tolerance * 2);

    m_kF = 1023 / rpmToTicksPer100ms(m_maxRPM);
    talon.config_kF(PID_SLOT, m_kF);

    // Configure motor deadband
    talon.configNeutralDeadband(MOTOR_DEADBAND);

    // Enable voltage compensation
    talon.configVoltageCompSaturation(MAX_VOLTAGE);
    talon.enableVoltageCompensation(true);

    // Configure MotionMagic values
    if (m_motionMagic) {
      talon.configMotionCruiseVelocity(rpmToTicksPer100ms(m_velocityRPM));
      talon.configMotionAcceleration(rpmToTicksPer100ms(m_accelerationRPMPerSec));
      talon.configMotionSCurveStrength(m_motionSmoothing);
    }
  }

  /**
   * Initializes Talon PID and MotionMagic parameters
   * <p>
   * Calls
   * {@link TalonPIDConfig#initializeTalonPID(BaseTalon, FeedbackDevice, boolean, boolean)}
   * with no limit switches
   * 
   * @param talon          Talon motor controller to apply settings to
   * @param feedbackDevice Feedback device to use for Talon PID
   */
  public void initializeTalonPID(BaseTalon talon, FeedbackDevice feedbackDevice) {
    initializeTalonPID(talon, feedbackDevice, false, false);
  }

  /**
   * Convert RPM to ticks per 100ms
   * 
   * @param rpm RPM
   * @return Value in ticks per 100ms
   */
  public double rpmToTicksPer100ms(double rpm) {
    return (rpm * m_ticksPerRotation) / 600;
  }

  /**
   * Convert ticks per 100ms to RPM
   * 
   * @param ticks Encoder ticks per 100ms
   * @return Value in RPM
   */
  public double ticksPer100msToRPM(double ticks) {
    return (ticks * 600) / m_ticksPerRotation;
  }

  /**
   * @return Sensor phase
   */
  public boolean getSensorPhase() {
    return m_sensorPhase;
  }

  /**
   * @return Whether motor is inverted or not
   */
  public boolean getInvertMotor() {
    return m_invertMotor;
  }

  /**
   * @return Proportional gain
   */
  public double getkP() {
    return m_kP;
  }

  /**
   * @return Integral gain
   */
  public double getkI() {
    return m_kI;
  }

  /**
   * @return Derivative gain
   */
  public double getkD() {
    return m_kD;
  }

  /**
   * @return Feed-forward gain
   */
  public double getkF() {
    return m_kF;
  }

  /**
   * @return PID loop tolerance
   */
  public double getTolerance() {
    return m_tolerance;
  }

  /**
   * @return Lower limit of mechanism
   */
  public double getLowerLimit() {
    return m_lowerLimit;
  }

  /**
   * @return Upper limit of mechanism
   */
  public double getUpperLimit() {
    return m_upperLimit;
  }

  /**
   * @return MotionMagic cruise velocity in RPM
   */
  public double getVelocityRPM() {
    return m_velocityRPM;
  }

  /**
   * @return MotionMagic acceleration in RPM per sec
   */
  public double getAccelerationRPMPerSec() {
    return m_accelerationRPMPerSec;
  }

  /**
   * @return MotionMagic smoothing factor
   */
  public int getMotionSmoothing() {
    return m_motionSmoothing;
  }
}
