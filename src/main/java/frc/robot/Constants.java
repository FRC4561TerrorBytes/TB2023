// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
    //public static final double FRONT_LEFT_STEER_OFFSET = 13; // FIXME Measure and set front left steer offset
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(127.0);

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_STEER_ENCODER = 22;
    public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean FRONT_RIGHT_TURN_MOTOR_INVERTED = true;
    //public static final double FRONT_RIGHT_STEER_OFFSET = 278.7; // FIXME Measure and set front right steer offset
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(77.0);

    public static final int BACK_LEFT_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_STEER_MOTOR = 8;
    public static final int BACK_LEFT_STEER_ENCODER = 24;
    public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean BACK_LEFT_TURN_MOTOR_INVERTED = true;
    //public static final double BACK_LEFT_STEER_OFFSET = 27.86; // FIXME Measure and set back left steer offset
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(234.0);

    public static final int BACK_RIGHT_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_STEER_ENCODER = 21;
    public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean BACK_RIGHT_TURN_MOTOR_INVERTED = true;
    //public static final double BACK_RIGHT_STEER_OFFSET = 60.73; // FIXME Measure and set back right steer offset
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(97.1);

    public static final double DRIVE_CURRENT_LIMIT = 80.0;
    public static final int TURN_CURRENT_LIMIT = 80;
    public static final double DRIVE_CURRENT_THRESHOLD = 120.0;
    public static final double DRIVE_CURRENT_TIME_THRESHOLD = 0.1; // seconds
    // current limits for drivetrain turn and steer motors
    // (Amps)

    public static final double DRIVE_MOTOR_CONVERSION_FACTOR = SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI / CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;

    public static final double TURN_MOTOR_KP = 1.0;
    public static final double TURN_MOTOR_KI = 0.0;
    public static final double TURN_MOTOR_KD = 0.1;
    public static final double TURN_MOTOR_MECHANICAL_EFFICIENCY = 1.0;
    public static final double TURN_MOTOR_TOLERANCE = Math.PI / 180;
    public static final double TURN_MOTOR_LOWER_LIMIT = 0.0;
    public static final double TURN_MOTOR_UPPER_LIMIT = 0.0;
    public static final boolean TURN_ENABLE_SOFT_LIMITS = false;
    public static final double TURN_MOTOR_CONVERSION_FACTOR = 2 * Math.PI * SdsModuleConfigurations.MK4I_L2.getSteerReduction();

    public static final double CAMERA_OFFSET_RIGHT = 0.11;
    public static final double CAMERA_OFFSET_BACK = 0.56;
    public static final double VISION_ROTATION_SCALING = 0.04;
    public static final double VISION_LATERAL_SCALING = 1;

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

}