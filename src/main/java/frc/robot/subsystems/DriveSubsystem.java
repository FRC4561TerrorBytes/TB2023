// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Pigeon gyro
  private final PigeonIMU m_pigeon = new PigeonIMU(Constants.PIGEON_ID);

  // Swerve Modules
  private final SwerveModule m_frontLeftModule = new SwerveModule(
      Constants.FRONT_LEFT_DRIVE_MOTOR,
      Constants.FRONT_LEFT_STEER_MOTOR,
      Constants.FRONT_LEFT_STEER_ENCODER,
      Constants.FRONT_LEFT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED,
      Constants.FRONT_LEFT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_frontRightModule = new SwerveModule(
      Constants.FRONT_RIGHT_DRIVE_MOTOR,
      Constants.FRONT_RIGHT_STEER_MOTOR,
      Constants.FRONT_RIGHT_STEER_ENCODER,
      Constants.FRONT_RIGHT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
      Constants.FRONT_RIGHT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_backLeftModule = new SwerveModule(
      Constants.BACK_LEFT_DRIVE_MOTOR,
      Constants.BACK_LEFT_STEER_MOTOR,
      Constants.BACK_LEFT_STEER_ENCODER,
      Constants.BACK_LEFT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED,
      Constants.BACK_LEFT_TURN_MOTOR_INVERTED);
  private final SwerveModule m_backRightModule = new SwerveModule(
      Constants.BACK_RIGHT_DRIVE_MOTOR,
      Constants.BACK_RIGHT_STEER_MOTOR,
      Constants.BACK_RIGHT_STEER_ENCODER,
      Constants.BACK_RIGHT_STEER_OFFSET,
      Constants.TURN_MOTOR_CONFIG,
      Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED,
      Constants.BACK_RIGHT_TURN_MOTOR_INVERTED);

  // Odometry
  private final SwerveDriveOdometry m_odometry;

  /**
   * The last set of ChassisSpeeds sent to or calcuated in drive(...). Note that
   * this set of speeds is <b>NOT</b> desaturated. It should only be used when
   * moderate and generally straight drive is expected. For example, during floor
   * intake or substation approach.
   */
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  public DriveSubsystem() {
    m_pigeon.setYaw(0.0);
    m_odometry = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      getModulePositions());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_lastChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getYaw()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
    final SwerveModuleState[] swerveModuleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(m_lastChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backLeftModule.setDesiredState(states[2]);
    m_backRightModule.setDesiredState(states[3]);

    updateOdometry();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  /**
   * Note that this set of speeds is <b>NOT</b> desaturated. It should only be
   * used when moderate and generally straight drive is expected. For example,
   * during floor intake or substation approach.
   */
  public FieldRelativeSpeeds getFieldRelativeSpeeds() {
    return FieldRelativeSpeeds.fromChassisSpeeds(m_lastChassisSpeeds, getRotation2d());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigeon.getYaw()), getModulePositions());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d position) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), position);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    };
  }

  public boolean onChargeStation() {
    return Math.abs(m_pigeon.getPitch()) > 20;
  }

  public boolean offPitchDown(){
    return Math.abs(m_pigeon.getPitch()) > 14;
  }

  public boolean onPitchDown() {
    // double[] angleRates = new double[3];
    // m_pigeon.getRawGyro(angleRates);
    // return onChargeStation() && angleRates[0] < -1;
    return Math.abs(m_pigeon.getPitch()) < 12;
  }

  // public Rotation2d getYaw(){
  //   return Rotation2d.fromDegrees(360 - m_pigeon.getYaw());
  // }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch", m_pigeon.getPitch());
    SmartDashboard.putBoolean("On Charge Station", onChargeStation());
    SmartDashboard.putBoolean("On Pitch Down", onPitchDown());
  }

  /**
   * A clone of {@link ChassisSpeeds} but for field relative speeds.
   */
  public static class FieldRelativeSpeeds {
    /**
     * Represents forward velocity w.r.t the field frame of reference. (Fwd is +)
     */
    public double vxMetersPerSecond;

    /**
     * Represents sideways velocity w.r.t the field frame of reference. (Left is +)
     */
    public double vyMetersPerSecond;

    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double omegaRadiansPerSecond;

    /**
     * Constructs a FieldRelativeSpeeds object.
     *
     * @param vxMetersPerSecond     Forward velocity.
     * @param vyMetersPerSecond     Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     */
    public FieldRelativeSpeeds(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
      this.vxMetersPerSecond = vxMetersPerSecond;
      this.vyMetersPerSecond = vyMetersPerSecond;
      this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
     * Lifted from CD with slight modification.
     */
    public static FieldRelativeSpeeds fromChassisSpeeds(final ChassisSpeeds chassisSpeeds,
        final Rotation2d robotAngle) {
      // Gets the field relative X and Y components of the robot's speed in the X axis
      double robotXXComp = robotAngle.getCos() * chassisSpeeds.vxMetersPerSecond;
      double robotXYComp = robotAngle.getSin() * chassisSpeeds.vxMetersPerSecond;

      // Gets the field relative X and Y components of the robot's speed in the Y axis
      double robotYXComp = robotAngle.getSin() * chassisSpeeds.vyMetersPerSecond;
      double robotYYComp = robotAngle.getCos() * chassisSpeeds.vyMetersPerSecond;

      // Adds the field relative X and Y components of the robot's X and Y speeds to
      // get the overall field relative X and Y speeds
      double fieldX = robotXXComp + robotYXComp;
      double fieldY = robotXYComp + robotYYComp;

      return new FieldRelativeSpeeds(fieldX, fieldY, chassisSpeeds.omegaRadiansPerSecond);
    }
  }
}