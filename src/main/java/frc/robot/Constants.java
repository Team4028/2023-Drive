// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.drive.RobotPhysics;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        // distance from the right to left wheels on the robot
        public static final double TRACK_WIDTH = 26;
        // distance from the front to back wheels on the robot
        public static final double WHEEL_BASE = 28;

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                Units.inchesToMeters(TRACK_WIDTH));

        public static final double SPEED_SCALE = 0.25;
        public static final double WHEEL_DIAMETER = 6.258;

        public static final double GEAR_RATIO = 7.5;

        public static final double MAX_VELOCITY =
            Units.feetToMeters(17.2); // Falcon
            // Units.feetToMeters(15.5); // NEO
            // Units.feetToMeters(14.4); // CIM

        // in radians per second
        public static final double MAX_ANGULAR_VELOCITY = Math.PI;// DriveConstants.MAX_VELOCITY /
        // Math.hypot(DriveConstants.TRACK_WIDTH / 2., DriveConstants.WHEEL_BASE / 2.);
        
        public static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            0,// MAX_ANGULAR_VELOCITY,
            TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER,
            GEAR_RATIO
        );

        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
                // TODO: get these from SysId
                0.0,
                0.0,
                0.0);

        public static final double NOMINAL_VOLTAGE = 12.;

        public static final String CAN_BUS = "rio";
    }

    public static final class AutonConstants {
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                PIDConstants.Theta.kP,
                0.,
                PIDConstants.Theta.kD,
                THETA_CONSTRAINTS);

        public static final TrajectoryConfig AUTON_TRAJECTORY_CONFIG = new TrajectoryConfig(
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_VELOCITY)
                        .setKinematics(DriveConstants.KINEMATICS);

        public static final PIDController DRIVE_CONTROLLER = new PIDController(PIDConstants.DriveController.kP, 0, 0);
    }

    public static final class SubsystemConstants {
        public static final int DRIVE_FL = 1;
        public static final int DRIVE_BL = 2;
        public static final int DRIVE_FR = 3;
        public static final int DRIVE_BR = 4;
    }

    public static final class PIDConstants {
        // TODO: get these from SysId
        public static final class Drive {
            public static final double kP = 0.01;
            public static final double kD = 0.02;
        }

        public static final class Turn {
            public static final double kP = 0.2;
        }

        public static final class Theta {
            public static final double kP = 6.5;
            public static final double kD = 0.15;
        }

        public static final class DriveController {
            public static final double kP = 8.;
        }
    }

    public static final class OIConstants {
        public static final int DRIVER = 0;
    }
}
