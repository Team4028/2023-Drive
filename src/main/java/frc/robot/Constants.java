// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        public static final double SPEED_SCALE = 0.25;
    }

    public static final class PIDConstants {
        public static final class Theta { // TODO: put in drivetrains
            public static final double kP = 6.3;
            public static final double kD = 0.05;
            public static final double[] gains = { kP, 0, kD };
        }
    }

    public static final class OIConstants {
        public static final int DRIVER = 0;
    }
}
