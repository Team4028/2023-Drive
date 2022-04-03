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
        public static final double kTrackWidth = 24.25;
        // distance from the front to back wheels on the robot
        public static final double kWheelBase = 24.25;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                Units.inchesToMeters(kTrackWidth));

        public static final double kSpeedScale = 0.5;
        public static final double kWheelDiameter = Units.inchesToMeters(6.258);

        public static final double kEncoderCPR = 4096.;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameter * Math.PI) / kEncoderCPR;

        public static final double kMaxSpeed = Units.feetToMeters(15.);

        public static final SimpleMotorFeedforward kFF = new SimpleMotorFeedforward(
                // TODO: get these from SysId
                0.0,
                0.0,
                0.0);

        public static final double kNominalVoltage = 12.;
    }

    public static final class AutonConstants {
        // in radians per second
        public static final double kMaxAngularSpeed = DriveConstants.kMaxSpeed /
                Math.hypot(DriveConstants.kTrackWidth / 2., DriveConstants.kWheelBase / 2.);

        public static final TrapezoidProfile.Constraints kThetaConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularSpeed);

        public static final ProfiledPIDController kThetaController = new ProfiledPIDController(
                PIDConstants.Theta.kP,
                0.,
                PIDConstants.Theta.kD,
                kThetaConstraints);

        public static final TrajectoryConfig kAutonTrajConfig = new TrajectoryConfig(
                DriveConstants.kMaxSpeed,
                DriveConstants.kMaxSpeed)
                        .setKinematics(DriveConstants.kDriveKinematics);

        public static final PIDController kDriveController = new PIDController(PIDConstants.DriveController.kP, 0, 0);
    }

    public static final class SubsystemConstants {
        public static final int kFL = 1;
        public static final int kBL = 2;
        public static final int kFR = 3;
        public static final int kBR = 4;
    }

    public static final class PIDConstants {
        // TODO: get these from SysId
        public static final class Drive {
            public static final double kP = 0.2;
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

    public static final class ControllerConstants {
        public static final double kDeadband = 0.05; // Jiggle room for the thumbsticks
        public static final double kSensitivity = 0.05;
        public static final double kTriggerDeadband = 0.01; // Jiggle room for the triggers
        public static final double kTriggerSensitivity = 0.6; // If the trigger is beyond this limit, say it has been
                                                              // pressed

        /* Button Mappings */
        public static final class Buttons {
            public static final int kA = 1;
            public static final int kB = 2;
            public static final int kX = 3;
            public static final int kY = 4;
            public static final int kLB = 5;
            public static final int kRB = 6;
            public static final int kBack = 7;
            public static final int kStart = 8;
            public static final int kLS = 9;
            public static final int kRS = 10;
        }

        /* Axis Mappings */
        public static final class Axes {
            public static final int kLeftX = 0;
            public static final int kLeftY = 1;
            public static final int kLeftTrigger = 2;
            public static final int kRightTrigger = 3;
            public static final int kRightX = 4;
            public static final int kRightY = 5;
        }
    }
}
