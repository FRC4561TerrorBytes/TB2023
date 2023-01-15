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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

  //Pigeon gyro
  private final PigeonIMU m_pigeon = new PigeonIMU(Constants.PIGEON_ID);

  //Swerve Modules
  private final SwerveModule m_frontLeftModule = new SwerveModule(
                                                    Constants.FRONT_LEFT_DRIVE_MOTOR , 
                                                    Constants.FRONT_LEFT_STEER_MOTOR, 
                                                    Constants.FRONT_LEFT_STEER_ENCODER,
                                                    Constants.FRONT_LEFT_STEER_OFFSET,
                                                    Constants.DRIVE_MOTOR_CONFIG,
                                                    Constants.TURN_MOTOR_CONFIG);
  private final SwerveModule m_frontRightModule = new SwerveModule(
                                                    Constants.FRONT_RIGHT_DRIVE_MOTOR, 
                                                    Constants.FRONT_RIGHT_STEER_MOTOR, 
                                                    Constants.FRONT_RIGHT_STEER_ENCODER, 
                                                    Constants.FRONT_RIGHT_STEER_OFFSET,
                                                    Constants.DRIVE_MOTOR_CONFIG,
                                                    Constants.TURN_MOTOR_CONFIG);
  private final SwerveModule m_backLeftModule = new SwerveModule(
                                                    Constants.BACK_LEFT_DRIVE_MOTOR, 
                                                    Constants.BACK_LEFT_STEER_MOTOR, 
                                                    Constants.BACK_LEFT_STEER_ENCODER, 
                                                    Constants.BACK_RIGHT_STEER_OFFSET,
                                                    Constants.DRIVE_MOTOR_CONFIG,
                                                    Constants.TURN_MOTOR_CONFIG);
  private final SwerveModule m_backRightModule = new SwerveModule(
                                                    Constants.BACK_RIGHT_DRIVE_MOTOR, 
                                                    Constants.BACK_RIGHT_STEER_MOTOR, 
                                                    Constants.BACK_RIGHT_STEER_ENCODER, 
                                                    Constants.BACK_RIGHT_STEER_OFFSET,
                                                    Constants.DRIVE_MOTOR_CONFIG,
                                                    Constants.TURN_MOTOR_CONFIG);
  

  //Odometry
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS, 
                                                                        Rotation2d.fromDegrees(m_pigeon.getYaw()), 
                                                                        new SwerveModulePosition[]{
                                                                          m_frontLeftModule.getPosition(),
                                                                          m_frontRightModule.getPosition(),
                                                                          m_backLeftModule.getPosition(),
                                                                          m_backRightModule.getPosition()
                                                                        });

  public DriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
    Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getYaw()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);
    // setModuleStates(states);
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_backLeftModule.setDesiredState(swerveModuleStates[2]);
    m_backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public Rotation2d geRotation2d() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigeon.getYaw()),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
        });
  }

  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return new Pose2d();
  }

  public void resetOdometry(Pose2d position) {
    
    // m_odometry.resetPosition(position, new Rotation2d());
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}