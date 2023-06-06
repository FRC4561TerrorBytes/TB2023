// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
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

  private final PIDController xController = new PIDController(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD);
  private final  PIDController yController = new PIDController(Constants.AUTO_Y_KP, Constants.AUTO_Y_KI, Constants.AUTO_Y_KD);
  private final  PIDController thetaController = new PIDController(Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI,
        Constants.AUTO_THETA_KD);

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
    var swerveModuleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
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

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigeon.getYaw()), getModulePositions());
    System.out.println("odometry pose " + getPose());
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

  public boolean onFlat(){
    return Math.abs(m_pigeon.getPitch()) < 5;
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

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.DRIVE_KINEMATICS,
             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );
 }
}