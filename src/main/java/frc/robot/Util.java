// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** Useful utility functions for drive. */
public final class Util {
    public static final double speedScale(double input, double base, double throttle) {
        return input * (base + throttle * (1.0 - base));
    }

    /**
     * Gets a command to control the
     * drivetrain to follow a path.
     * 
     * @param traj Trajectory to follow.
     */
    public static final SequentialCommandGroup getTrajectoryCommand(Trajectory traj) {
        return new RamseteCommand(
                traj,
                Drivetrain.getInstance()::getPoseMeters,
                new RamseteController(),
                DriveConstants.kFF,
                DriveConstants.kDriveKinematics,
                Drivetrain.getInstance()::getWheelSpeeds,
                AutonConstants.kDriveController,
                AutonConstants.kDriveController,
                Drivetrain.getInstance()::driveVolts,
                Drivetrain.getInstance()).andThen(() -> Drivetrain.getInstance().drive(0, 0, 0));
    }
}
