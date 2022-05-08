// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.BeakDifferentialDrivetrain;

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
    public static final SequentialCommandGroup getTrajectoryCommand(PathPlannerTrajectory traj, BeakDifferentialDrivetrain drivetrain) {
        return new RamseteCommand(
                traj,
                drivetrain::getPoseMeters,
                new RamseteController(),
                DriveConstants.FEED_FORWARD,
                DriveConstants.DRIVE_KINEMATICS,
                drivetrain::getWheelSpeeds,
                AutonConstants.DRIVE_CONTROLLER,
                AutonConstants.DRIVE_CONTROLLER,
                drivetrain::driveVolts,
                drivetrain
            ).andThen(() -> drivetrain.drive(0, 0, 0));
    }

    /**
     * Convert native units of an encoder
     * into a distance in inches travelled
     * by a wheel.
     * 
     * @param nu Native units to convert.
     * @param cpr Encoder ticks per motor revolution.
     * @param gearRatio Motor rotations per gearbox shaft rotation.
     * @param wheelDiameter Diameter of the wheel, in inches.
     * 
     * @return Wheel travel in meters.</p>
     * 
     * Example:
     * TalonSRX reports a velocity of 8192 NU/100ms, with a 10.75:1 gear ratio, on a 4-inch wheel.
     * 
     * <pre>
     * {@code
     * double travelMeters = Util.NUtoMeters(8192, 4096, 10.75, 4)
     * }
     * </pre>
     * - 2 rotations of the motor (8192NU travel / 4096NU CPR)</p>
     * - ~.186 rotations of the wheel (2 motor rotations / 10.75 gear ratio)</p>
     * - ~2.34 in travel (.186 shaft rotations * (PI * 4 in))
    */
    public static double NUtoMeters(double nu, double cpr, double gearRatio, double wheelDiameter){
        double motorRotations = (double)nu / cpr; // How many rotations of the motor in this amount of NU.
        double wheelRotations = motorRotations / gearRatio; // How many rotations of the gearbox (wheel).
        double positionMeters = wheelRotations * (Math.PI * Units.inchesToMeters(wheelDiameter)); // The distance travelled by the wheel.
        return positionMeters;
    }

    public static double NUtoMeters(double nu) {
        return NUtoMeters(nu, DriveConstants.ENCODER_CPR, DriveConstants.GEAR_RATIO, Units.metersToInches(DriveConstants.WHEEL_DIAMETER));
    }
}
