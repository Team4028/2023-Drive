// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PIDConstants;
import frc.robot.utilities.drive.RobotPhysics;
import frc.robot.utilities.drive.swerve.epic.BeakEpicSwerveDrivetrain;
import frc.robot.utilities.drive.swerve.SdsModuleConfiguration;
import frc.robot.utilities.drive.swerve.SdsModuleConfigurations;
import frc.robot.utilities.drive.swerve.SwerveDrivetrainConfiguration;
import frc.robot.utilities.drive.swerve.epic.EpicSwerveModuleConfiguration;
import frc.robot.utilities.units.AngularVelocity;
import frc.robot.utilities.units.Distance;
import frc.robot.utilities.units.Velocity;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class EpicSwerveDrivetrain extends BeakEpicSwerveDrivetrain {
    private static final double DRIVE_kP = 0.0125;
    private static final double TURN_kP = 0.4;
    private static final double TURN_kD = 0.2;

    private static final double AUTON_kP = 3.;
    private static final double[] AUTON_DRIVE_GAINS = { AUTON_kP, 0., 0. };

    private static final int PIGEON2_ID = 1;
    private static final String CAN_BUS = "DriveSubsystem";

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            (0.19 + 0.225 + 0.214 + 0.2256) / 4.0,
            (2.2565 + 2.2785 + 2.2754 + 2.291) / 4.0,
            (0.277 + 0.31) / 2.0);

    private static final SdsModuleConfiguration CONFIGURATION = SdsModuleConfigurations.MK4I_L2;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(16.3);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(26.);
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(28.);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            new AngularVelocity(),
            TRACK_WIDTH,
            WHEEL_BASE,
            CONFIGURATION.wheelDiameter,
            CONFIGURATION.driveGearRatio,
            FEED_FORWARD);

    private static EpicSwerveDrivetrain m_instance;

    private Field2d m_field = new Field2d();

    // TODO: get offsets
    // TODO: organize this
    private static final int FL_DRIVE_ID = 2;
    private static final int FL_TURN_ID = 1;
    private static final int FL_ENCODER_ID = 1; // SHOULD BE 9
    private static final double FL_OFFSET = -Units.degreesToRadians(139.2);
    private static final Translation2d FL_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
            TRACK_WIDTH.getAsMeters() / 2); // TODO: Please God BeakTranslation2d

    private static final int FR_DRIVE_ID = 4;
    private static final int FR_TURN_ID = 3;
    private static final int FR_ENCODER_ID = 2; // SHOULD BE 10
    private static final double FR_OFFSET = -Math.toRadians(322.3);
    private static final Translation2d FR_LOCATION = new Translation2d(WHEEL_BASE.getAsMeters() / 2,
            -TRACK_WIDTH.getAsMeters() / 2);

    private static final int BL_DRIVE_ID = 6;
    private static final int BL_TURN_ID = 5;
    private static final int BL_ENCODER_ID = 3; // SHOULD BE 11
    private static final double BL_OFFSET = -Math.toRadians(106.1);
    private static final Translation2d BL_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
            TRACK_WIDTH.getAsMeters() / 2);

    private static final int BR_DRIVE_ID = 8;
    private static final int BR_TURN_ID = 7;
    private static final int BR_ENCODER_ID = 4; // SHOULD BE 12
    private static final double BR_OFFSET = -Math.toRadians(234.1);
    private static final Translation2d BR_LOCATION = new Translation2d(-WHEEL_BASE.getAsMeters() / 2,
            -TRACK_WIDTH.getAsMeters() / 2);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 40.0;

    private static final int TURN_CURRENT_LIMIT = 20;
    private static final int DRIVE_SUPPLY_LIMIT = 60;
    private static final int DRIVE_STATOR_LIMIT = 80;

    private final static WPI_Pigeon2 m_gyro = new WPI_Pigeon2(PIGEON2_ID, CAN_BUS);

    private static final SwerveDrivetrainConfiguration DRIVE_CONFIG = new SwerveDrivetrainConfiguration(
            DRIVE_kP,
            TURN_kP,
            TURN_kD,
            ALLOWED_CLOSED_LOOP_ERROR,
            TURN_CURRENT_LIMIT,
            DRIVE_SUPPLY_LIMIT,
            DRIVE_STATOR_LIMIT,
            CAN_BUS,
            FEED_FORWARD,
            CONFIGURATION);

    private static EpicSwerveModuleConfiguration m_frontLeftConfig = new EpicSwerveModuleConfiguration(
            FL_DRIVE_ID,
            FL_TURN_ID,
            FL_ENCODER_ID,
            FL_OFFSET,
            FL_LOCATION,
            DRIVE_CONFIG);

    private static EpicSwerveModuleConfiguration m_frontRightConfig = new EpicSwerveModuleConfiguration(
            FR_DRIVE_ID,
            FR_TURN_ID,
            FR_ENCODER_ID,
            FR_OFFSET,
            FR_LOCATION,
            DRIVE_CONFIG);

    private static EpicSwerveModuleConfiguration m_backLeftConfig = new EpicSwerveModuleConfiguration(
            BL_DRIVE_ID,
            BL_TURN_ID,
            BL_ENCODER_ID,
            BL_OFFSET,
            BL_LOCATION,
            DRIVE_CONFIG);

    private static EpicSwerveModuleConfiguration m_backRightConfig = new EpicSwerveModuleConfiguration(
            BR_DRIVE_ID,
            BR_TURN_ID,
            BR_ENCODER_ID,
            BR_OFFSET,
            BR_LOCATION,
            DRIVE_CONFIG);

    public EpicSwerveDrivetrain() {
        super(
                PHYSICS,
                m_gyro,
                false,
                PIDConstants.Theta.gains,
                AUTON_DRIVE_GAINS,
                m_frontLeftConfig,
                m_frontRightConfig,
                m_backLeftConfig,
                m_backRightConfig);
    }

    public static EpicSwerveDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new EpicSwerveDrivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("FL angle", Math.toDegrees(m_modules.get(0).getTurningEncoderRadians()));
        SmartDashboard.putNumber("FR angle", Math.toDegrees(m_modules.get(1).getTurningEncoderRadians()));
        SmartDashboard.putNumber("BL angle", Math.toDegrees(m_modules.get(2).getTurningEncoderRadians()));
        SmartDashboard.putNumber("BR angle", Math.toDegrees(m_modules.get(3).getTurningEncoderRadians()));
        m_field.setRobotPose(getPoseMeters());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());
    }
}
