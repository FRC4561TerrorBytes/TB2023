// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory {

    DriveSubsystem m_driveSubsystem;
    PathPlannerTrajectory m_pathPlannerTrajectory;
    PPSwerveControllerCommand m_swerveControllerCommand;

    public AutoTrajectory(DriveSubsystem driveSubsystem, String autoPathName, double maxSpeedMetersPerSec, double maxAccelerationMetersPerSecSquared){
        this.m_driveSubsystem = driveSubsystem;

        m_pathPlannerTrajectory = PathPlanner.loadPath(autoPathName, maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared);

        // Auto PID Controllers
        PIDController xController = new PIDController(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD);
        PIDController yController = new PIDController(Constants.AUTO_Y_KP, Constants.AUTO_Y_KI, Constants.AUTO_Y_KD);
        PIDController thetaController = new PIDController(Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI, Constants.AUTO_THETA_KD);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_swerveControllerCommand = new PPSwerveControllerCommand(
            m_pathPlannerTrajectory, 
            m_driveSubsystem::getPose,
            Constants.DRIVE_KINEMATICS,
            xController, 
            yController, 
            thetaController,
            m_driveSubsystem::setModuleStates, 
            m_driveSubsystem);
    }

    public AutoTrajectory(DriveSubsystem driveSubsystem, double maxSpeedMetersPerSec, double maxAccelerationMetersPerSecSquared, 
                            Translation2d targetTranslation, Rotation2d targetRotation){
        m_driveSubsystem = driveSubsystem;
        m_pathPlannerTrajectory = PathPlanner.generatePath(new PathConstraints(maxSpeedMetersPerSec, maxAccelerationMetersPerSecSquared),
                                                         new PathPoint(new Translation2d(), m_driveSubsystem.getRotation2d(), new Rotation2d()),
                                                         new PathPoint(targetTranslation, new Rotation2d(), targetRotation));

        // Auto PID Controllers
        PIDController xController = new PIDController(Constants.AUTO_X_KP, Constants.AUTO_X_KI, Constants.AUTO_X_KD);
        xController.setTolerance(0.01);
        PIDController yController = new PIDController(Constants.AUTO_Y_KP, Constants.AUTO_Y_KI, Constants.AUTO_Y_KD);
        yController.setTolerance(0.01);
        PIDController thetaController = new PIDController(Constants.AUTO_THETA_KP, Constants.AUTO_THETA_KI, Constants.AUTO_THETA_KD);
        thetaController.setTolerance(0.1);

        m_swerveControllerCommand = new PPSwerveControllerCommand(
            m_pathPlannerTrajectory, 
            m_driveSubsystem::getPose,
            Constants.DRIVE_KINEMATICS,
            xController, 
            yController, 
            thetaController, 
            m_driveSubsystem::setModuleStates, 
            m_driveSubsystem);
    }

    public void resetOdometry(){
        m_driveSubsystem.resetOdometry(m_pathPlannerTrajectory.getInitialHolonomicPose());
    }

    public Command getCommandAndStop(){
        return new InstantCommand(() -> resetOdometry(), m_driveSubsystem).andThen(
            m_swerveControllerCommand.withTimeout(m_pathPlannerTrajectory.getTotalTimeSeconds()).andThen(() -> m_driveSubsystem.stop())
        );
    }
}