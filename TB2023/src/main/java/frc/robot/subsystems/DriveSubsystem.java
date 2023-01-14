// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

  //Pigeon gyro
  private final PigeonIMU m_pigeon = new PigeonIMU(Constants.PIGEON_ID);

  //Swerve Modules
  private final SwerveModule m_frontLeftModule = new SwerveModule(5, 6, 23, -Math.toRadians(127.0));
  private final SwerveModule m_frontRightModule = new SwerveModule(3, 4, 22 , -Math.toRadians(77.0));
  private final SwerveModule m_backLeftModule = new SwerveModule(7, 8, 24, -Math.toRadians(234.0));
  private final SwerveModule m_backRightModule = new SwerveModule(5, 6, 23, -Math.toRadians(97.1));
  

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
