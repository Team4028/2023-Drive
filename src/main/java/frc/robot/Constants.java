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

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        // distance from the right to left wheels on the robot
        public static final double TRACK_WIDTH = 24.25;
        // distance from the front to back wheels on the robot
        public static final double WHEEL_BASE = 24.25;

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                Units.inchesToMeters(TRACK_WIDTH));

        public static final double SPEED_SCALE = 0.25;
        public static final double WHEEL_DIAMETER = .15895;//Units.inchesToMeters(6.258);

        public static final double ENCODER_CPR = 4096.;
        public static final double GEAR_RATIO = 7.5;
        public static final double ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER * Math.PI) / ENCODER_CPR;

        public static final double MAX_VELOCITY = 4.572;//Units.feetToMeters(15.);

        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
                // TODO: get these from SysId
                0.0,
                0.0,
                0.0);

        public static final double NOMINAL_VOLTAGE = 12.;
    }

    public static final class AutonConstants {
        // in radians per second
        public static final double MAX_ANGULAR_VELOCITY = 2.0;//DriveConstants.MAX_VELOCITY /
                // Math.hypot(DriveConstants.TRACK_WIDTH / 2., DriveConstants.WHEEL_BASE / 2.);

        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                PIDConstants.Theta.kP,
                0.,
                PIDConstants.Theta.kD,
                THETA_CONSTRAINTS);

        public static final TrajectoryConfig AUTON_TRAJECTORY_CONFIG = new TrajectoryConfig(
                DriveConstants.MAX_VELOCITY,
                DriveConstants.MAX_VELOCITY)
                        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

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
            public static final double kD = 0.0;
        }

        public static final class Theta {
            public static final double kP = 3.5;
            public static final double kD = 0.1;
        }

        public static final class DriveController {
            public static final double kP = 8.;
        }
    }

    public static final class OIConstants {
        public static final int DRIVER = 0;
    }

    public static final class ControllerConstants {
        public static final double THUMBSTICK_DEADBAND = 0.05; // Jiggle room for the thumbsticks
        public static final double THUMBSTICK_SENSITIVITY = 0.05;
        public static final double TRIGGER_DEADBAND = 0.01; // Jiggle room for the triggers
        public static final double TRIGGER_SENSITIVITY = 0.6; // If the trigger is beyond this limit, say it has been
                                                              // pressed

        /* Button Mappings */
        public static final class Buttons {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int LEFT_BUMPER = 5;
            public static final int RIGHT_BUMPER = 6;
            public static final int BACK = 7;
            public static final int START = 8;
            public static final int LEFT_STICK = 9;
            public static final int RIGHT_STICK = 10;
        }

        /* Axis Mappings */
        public static final class Axes {
            public static final int LEFT_X = 0;
            public static final int LEFT_Y = 1;
            public static final int LEFT_TRIGGER = 2;
            public static final int RIGHT_TRIGGER = 3;
            public static final int RIGHT_X = 4;
            public static final int RIGHT_Y = 5;
        }
    }
}
