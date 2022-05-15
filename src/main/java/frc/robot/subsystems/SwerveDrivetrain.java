// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.utilities.drive.swerve.BeakMk4iSwerveDrivetrain;
import frc.robot.utilities.drive.swerve.SdsModuleConfigurations;
import frc.robot.utilities.drive.swerve.SwerveModuleConfiguration;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;

/** Add your docs here. */
public class SwerveDrivetrain extends BeakMk4iSwerveDrivetrain {
    // TODO: get offsets
    private static final int FL_DRIVE_ID = 2;
    private static final int FL_TURN_ID = 1;
    private static final int FL_ENCODER_ID = 1; // SHOULD BE 9
    private static final double FL_OFFSET = -Math.toRadians(0.);

    private static final int FR_DRIVE_ID = 4;
    private static final int FR_TURN_ID = 3;
    private static final int FR_ENCODER_ID = 2; // SHOULD BE 10
    private static final double FR_OFFSET = -Math.toRadians(0.);

    private static final int BL_DRIVE_ID = 6;
    private static final int BL_TURN_ID = 5;
    private static final int BL_ENCODER_ID = 3; // SHOULD BE 11
    private static final double BL_OFFSET = -Math.toRadians(0.);

    private static final int BR_DRIVE_ID = 8;
    private static final int BR_TURN_ID = 7;
    private static final int BR_ENCODER_ID = 4; // SHOULD BE 12
    private static final double BR_OFFSET = -Math.toRadians(0.);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 40.0;

    private static SwerveModuleConfiguration m_frontLeftConfig = new SwerveModuleConfiguration(
            FL_DRIVE_ID,
            FL_TURN_ID,
            FL_ENCODER_ID,
            FL_OFFSET,
            PIDConstants.Drive.kP,
            PIDConstants.Turn.kP,
            ALLOWED_CLOSED_LOOP_ERROR,
            DriveConstants.CAN_BUS,
            DriveConstants.FEED_FORWARD,
            SdsModuleConfigurations.MK4_L2
        );

    private static SwerveModuleConfiguration m_frontRightConfig = new SwerveModuleConfiguration(
            FR_DRIVE_ID,
            FR_TURN_ID,
            FR_ENCODER_ID,
            FR_OFFSET,
            PIDConstants.Drive.kP,
            PIDConstants.Turn.kP,
            ALLOWED_CLOSED_LOOP_ERROR,
            DriveConstants.CAN_BUS,
            DriveConstants.FEED_FORWARD,
            SdsModuleConfigurations.MK4_L2
        );

    private static SwerveModuleConfiguration m_backLeftConfig = new SwerveModuleConfiguration(
            BL_DRIVE_ID,
            BL_TURN_ID,
            BL_ENCODER_ID,
            BL_OFFSET,
            PIDConstants.Drive.kP,
            PIDConstants.Turn.kP,
            ALLOWED_CLOSED_LOOP_ERROR,
            DriveConstants.CAN_BUS,
            DriveConstants.FEED_FORWARD,
            SdsModuleConfigurations.MK4_L2
        );

    private static SwerveModuleConfiguration m_backRightConfig = new SwerveModuleConfiguration(
            BR_DRIVE_ID,
            BR_TURN_ID,
            BR_ENCODER_ID,
            BR_OFFSET,
            PIDConstants.Drive.kP,
            PIDConstants.Turn.kP,
            ALLOWED_CLOSED_LOOP_ERROR,
            DriveConstants.CAN_BUS,
            DriveConstants.FEED_FORWARD,
            SdsModuleConfigurations.MK4_L2
        );

    public SwerveDrivetrain() {
        super(
            m_frontLeftConfig,
            m_frontRightConfig,
            m_backLeftConfig,
            m_backRightConfig,
            DriveConstants.PHYSICS,
            1
        );
    }
}
