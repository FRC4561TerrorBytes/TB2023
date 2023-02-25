// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int PIGEON_ID = 20;

  public static final int FALCON_500_MAX_RPM = 6380;
  public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;
  public static final int NEO_MAX_RPM = 5676;
  public static final int NEO_TICKS_PER_ROTATION = 4096;

  public static final double MAX_VOLTAGE = 12.0;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.53;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.53;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_500_MAX_RPM / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public static final int FRONT_LEFT_DRIVE_MOTOR = 5;
  public static final int FRONT_LEFT_STEER_MOTOR = 6;
  public static final int FRONT_LEFT_STEER_ENCODER = 23;
  public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean FRONT_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_LEFT_STEER_OFFSET = 13; // FIXME Measure and set front left steer offset

  public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
  public static final int FRONT_RIGHT_STEER_MOTOR = 4;
  public static final int FRONT_RIGHT_STEER_ENCODER = 22;
  public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean FRONT_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double FRONT_RIGHT_STEER_OFFSET = 278.7; // FIXME Measure and set front right steer offset

  public static final int BACK_LEFT_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_STEER_MOTOR = 8;
  public static final int BACK_LEFT_STEER_ENCODER = 24;
  public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = true;
  public static final boolean BACK_LEFT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_LEFT_STEER_OFFSET = 27.86; // FIXME Measure and set back left steer offset

  public static final int BACK_RIGHT_DRIVE_MOTOR = 1;
  public static final int BACK_RIGHT_STEER_MOTOR = 2;
  public static final int BACK_RIGHT_STEER_ENCODER = 21;
  public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
  public static final boolean BACK_RIGHT_TURN_MOTOR_INVERTED = true;
  public static final double BACK_RIGHT_STEER_OFFSET = 60.73; // FIXME Measure and set back right steer offset

  public static final double DRIVE_CURRENT_LIMIT = 80.0;
  public static final int TURN_CURRENT_LIMIT = 80;
  public static final double DRIVE_CURRENT_THRESHOLD = 120.0;
  public static final double DRIVE_CURRENT_TIME_THRESHOLD = 0.1; // seconds
  // current limits for drivetrain turn and steer motors
  // (Amps)

  public static final double TURN_MOTOR_KP = 1.0;
  public static final double TURN_MOTOR_KI = 0.0;
  public static final double TURN_MOTOR_KD = 0.1;
  public static final double TURN_MOTOR_MECHANICAL_EFFICIENCY = 1.0;
  public static final double TURN_MOTOR_TOLERANCE = Math.PI / 180;
  public static final double TURN_MOTOR_LOWER_LIMIT = 0.0;
  public static final double TURN_MOTOR_UPPER_LIMIT = 0.0;
  public static final boolean TURN_ENABLE_SOFT_LIMITS = false;
  public static final double TURN_MOTOR_CONVERSION_FACTOR = 2 * Math.PI
      * SdsModuleConfigurations.MK4I_L2.getSteerReduction();
  public static final double DRIVE_MOTOR_CONVERSION_FACTOR = SdsModuleConfigurations.MK4I_L2.getDriveReduction()
      * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI
      / CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;

  // fyi: offset right means lateral offset from center
  public static final double LEFT_CAMERA_OFFSET_RIGHT = .28;
  public static final double RIGHT_CAMERA_OFFSET_RIGHT = -.28;
  public static final double LEFT_CAMERA_OFFSET_BACK = .57;
  public static final double RIGHT_CAMERA_OFFSET_BACK = .57;
  public static final double VISION_ROTATION_SCALING = 0.1;
  public static final double VISION_LATERAL_SCALING = 2;

  public static final double VISION_ROTATION_FLOOR_CLAMP = 2;
  public static final double VISION_ROTATION_CEILING_CLAMP = 50;
  public static final double VISION_ROTATION_DEADBAND = 6;
  public static final double VISION_ROTATION_TOLERANCE = 3;

  public static final double VISION_LATERAL_FLOOR_CLAMP = 0.2;
  public static final double VISION_LATERAL_CEILING_CLAMP = 0.5;
  public static final double VISION_LATERAL_DEADBAND = 0.1;
  public static final double VISION_LATERAL_TOLERANCE = 0.05;

  public static final double VISION_END_DISTANCE = 0.45;
  public static final double VISION_FORWARD_FLOOR_CLAMP = 0.2;
  public static final double VISION_FORWARD_CEILING_CLAMP = 1;

  public static final SparkPIDConfig TURN_MOTOR_CONFIG = new SparkPIDConfig(
      false,
      NEO_MAX_RPM,
      TURN_MOTOR_KP,
      TURN_MOTOR_KI,
      TURN_MOTOR_KD,
      TURN_MOTOR_MECHANICAL_EFFICIENCY,
      TURN_MOTOR_TOLERANCE,
      TURN_MOTOR_LOWER_LIMIT,
      TURN_MOTOR_UPPER_LIMIT,
      TURN_ENABLE_SOFT_LIMITS,
      NEO_MAX_RPM,
      NEO_MAX_RPM,
      TURN_MOTOR_CONVERSION_FACTOR);

  public static final int ELBOW_MOTOR = 11;
  public static final int SHOULDER_MOTOR = 10;

  public static final double ELBOW_ROTATIONS_PER_DEGREE = 5.0 / 36.0;
  public static final double ELBOW_MAX_VOLTAGE_FF = 1.25;
  public static final double ELBOW_PROPORTIONAL_GAIN_SLOT_0 = 0.0085;
  public static final double ELBOW_PROPORTIONAL_GAIN_SLOT_1 = 0.001;
  public static final double ELBOW_DERIVATIVE_GAIN = 0.0001;
  public static final double ELBOW_CRUISE_VELOCITY_DEG_PER_SEC = 20.0;
  /** deg/sec * sec/min = deg/min ... deg/min * rot/deg = rot/min = RPM */
  public static final double ELBOW_CRUISE_VELOCITY_RPM = ELBOW_CRUISE_VELOCITY_DEG_PER_SEC * 60.0
      * ELBOW_ROTATIONS_PER_DEGREE;
  public static final double ELBOW_SECONDS_TO_CRUISE_RPM = 1.25;
  /** RPM/sec */
  public static final double ELBOW_PEAK_ACCELERATION = ELBOW_CRUISE_VELOCITY_RPM
      / ELBOW_SECONDS_TO_CRUISE_RPM;
  public static final double ELBOW_TOLERANCE = ELBOW_ROTATIONS_PER_DEGREE / 2.0;

  public static final double SHOULDER_ROTATIONS_PER_DEGREE = 25.0 / 27.0;
  // needs fixing; moves to fast and with to much force.
  public static final double SHOULDER_MAX_VOLTAGE_FF = 0.5;
  public static final double SHOULDER_PROPORTIONAL_GAIN_SLOT_0 = 0.02;
  public static final double SHOULDER_PROPORTIONAL_GAIN_SLOT_1 = 0.02;
  public static final double SHOULDER_DERIVATIVE_GAIN = 0.0;
  public static final double SHOULDER_CRUISE_VELOCITY_DEG_PER_SEC = 20.0;
  /** deg/sec * sec/min = deg/min ... deg/min * rot/deg = rot/min = RPM */
  public static final double SHOULDER_CRUISE_VELOCITY_RPM = SHOULDER_CRUISE_VELOCITY_DEG_PER_SEC * 60.0
      * SHOULDER_ROTATIONS_PER_DEGREE;
  public static final double SHOULDER_SECONDS_TO_CRUISE_RPM = 1.25;
  /** RPM/sec */
  public static final double SHOULDER_PEAK_ACCELERATION = SHOULDER_CRUISE_VELOCITY_RPM
      / SHOULDER_SECONDS_TO_CRUISE_RPM;
  public static final double SHOULDER_TOLERANCE = SHOULDER_ROTATIONS_PER_DEGREE / 2.0;

  public static final double ARM_MOTORS_NEUTRAL_DEADBAND = 0.0001;
  public static final double ARM_MOTORS_INTERGRAL_ZONE = 10000.0;
  public static final double ARM_MOTORS_CLOSED_LOOP_PEAK_OUTPUT = 0.5;

  public static final double SHOULDER_ZERO_OFFSET = /* measure this */ 111.5;
  public static final double ELBOW_ZERO_OFFSET = /* measure this */ -42.5 - SHOULDER_ZERO_OFFSET + 90.0;

  public static final int LEFT_INTAKE_MOTOR = 12;
  public static final int RIGHT_INTAKE_MOTOR = 13;

  public static final double INTAKE_SPEED = 0.1;

}