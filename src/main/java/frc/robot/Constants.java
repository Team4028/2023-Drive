// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        public static final double SPEED_SCALE = 0.25;
    }

    public static final class AutonConstants {
        public static final PIDController DRIVE_CONTROLLER = new PIDController(PIDConstants.DriveController.kP, 0, 0);
    }

    public static final class PIDConstants {
        public static final class Theta {
            public static final double kP = 4.5;
            public static final double kD = 0.15;
            public static final double[] gains = { kP, 0, kD };
        }

        public static final class DriveController {
            public static final double kP = 5.;
        }
    }

    public static final class OIConstants {
        public static final int DRIVER = 0;
    }
}
