// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;

/**
 * Automates the configuration of Spark PID and Smart Motion paramaters
 */
public class SparkPIDConfig {
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MIN_TOLERANCE = 1.0;
  private static final int PID_SLOT = 0;

  private boolean m_smartMotion = false;
  private boolean m_enableSoftLimits = true;

  private boolean m_invertMotor = false;
  private double m_maxRPM = 0.0;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;
  private double m_conversionFactor = 1.0;

  private double m_velocityRPM = 1.0;
  private double m_accelerationRPMPerSec = 1.0;

  private SparkMaxPIDController m_pidController;

  public SparkPIDConfig(boolean invertMotor, double maxRPM,
                        double kP, double kI, double kD, double mechanicalEfficiency, double tolerance, 
                        double lowerLimit, double upperLimit, boolean enableSoftLimits,
                        double velocityRPM, double accelerationRPMPerSec) {
    this(invertMotor, maxRPM, kP, kI, kD, mechanicalEfficiency, tolerance, lowerLimit, upperLimit, enableSoftLimits, velocityRPM, accelerationRPMPerSec, 1);
  }

  /**
   * Create a SparkPIDConfig, with MotionMagic parameters
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
  public SparkPIDConfig(boolean invertMotor, double maxRPM,
      double kP, double kI, double kD, double mechanicalEfficiency, double tolerance,
      double lowerLimit, double upperLimit, boolean enableSoftLimits,
      double velocityRPM, double accelerationRPMPerSec, double conversionFactor) {
    this.m_invertMotor = invertMotor;
    this.m_maxRPM = maxRPM * MathUtil.clamp(mechanicalEfficiency, 0.0, 1.0);
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_conversionFactor = conversionFactor;
    this.m_enableSoftLimits = enableSoftLimits;

    this.m_velocityRPM = velocityRPM;
    this.m_accelerationRPMPerSec = accelerationRPMPerSec;

    this.m_smartMotion = true;
  }

  /**
   * Initializes Talon PID and Smart Motion parameters
   * 
   * @param spark              Spark motor controller to apply settings to
   * @param feedbackSensor     Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public SparkMaxPIDController initializeSparkPID(CANSparkMax spark,
      boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Reset Talon to default
    spark.restoreFactoryDefaults();

    // Get PID controller
    m_pidController = spark.getPIDController();

    spark.getEncoder().setPositionConversionFactor(m_conversionFactor);

    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_upperLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_lowerLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch)
      spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
    if (reverseLimitSwitch)
      spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);

    // Invert motor if required
    spark.setInverted(m_invertMotor);

    // Configure PID values
    m_pidController.setP(m_kP, PID_SLOT);
    m_pidController.setI(m_kI, PID_SLOT);
    m_pidController.setD(m_kD, PID_SLOT);
    m_pidController.setOutputRange(-1.0, +1.0);

    m_pidController.setIZone(m_tolerance * 2, PID_SLOT);

    m_kF = 1 / m_maxRPM;
    m_pidController.setFF(m_kF, PID_SLOT);

    // Enable voltage compensation
    spark.enableVoltageCompensation(MAX_VOLTAGE);

    // Configure Smart Motion values
    if (m_smartMotion) {
      m_pidController.setSmartMotionMaxVelocity(m_velocityRPM, PID_SLOT);
      m_pidController.setSmartMotionMaxAccel(m_accelerationRPMPerSec, PID_SLOT);
    }

    return m_pidController;
  }

  /**
   * Initializes Talon PID and MotionMagic parameters
   * <p>
   * Calls
   * {@link TalonPIDConfig#initializeTalonPID(CANSparkMax, FeedbackDevice, boolean, boolean)}
   * with no limit switches
   * 
   * @param spark          Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public SparkMaxPIDController initializeSparkPID(CANSparkMax spark) {
    return initializeSparkPID(spark, false, false);
  }
}
