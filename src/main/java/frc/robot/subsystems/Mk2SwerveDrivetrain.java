// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PIDConstants;
import frc.robot.utilities.drive.RobotPhysics;
import frc.robot.utilities.drive.swerve.BeakSwerveDrivetrain;
import frc.robot.utilities.drive.swerve.SdsModuleConfiguration;
import frc.robot.utilities.drive.swerve.SdsModuleConfigurations;
import frc.robot.utilities.drive.swerve.SwerveDrivetrainConfiguration;
import frc.robot.utilities.drive.swerve.SwerveModuleConfiguration;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Mk2SwerveDrivetrain extends BeakSwerveDrivetrain {
    protected boolean m_gyroInverted = true;
    private static Mk2SwerveDrivetrain m_instance;
    private Field2d m_field = new Field2d();

    private static final double DRIVE_kP = 0.0001;
    private static final double TURN_kP = 0.45;

    private static final double AUTON_kP = 1.;
    private static final double[] AUTON_DRIVE_GAINS = { AUTON_kP, 0., 0. };

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            0,
            0,
            0);

    private static final SdsModuleConfiguration CONFIGURATION = SdsModuleConfigurations.MK2_L2;

    private static final String CAN_BUS = "";

    private static final double MAX_VELOCITY = Units.feetToMeters(12.0);

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 23.5;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 21.5;

    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            0., // TEMP
            TRACK_WIDTH,
            WHEEL_BASE,
            Units.metersToInches(CONFIGURATION.wheelDiameter),
            CONFIGURATION.driveGearRatio,
            FEED_FORWARD);

    // TODO: get offsets
    // TODO: organize this
    private static final int FL_DRIVE_ID = 4;
    private static final int FL_TURN_ID = 3;
    private static final int FL_ENCODER_ID = 1; // SHOULD BE 9
    private static final double FL_OFFSET = -Units.degreesToRadians(19.4);//244.9 + 180.); //324.4 + 180.0);

    private static final int FR_DRIVE_ID = 2;
    private static final int FR_TURN_ID = 1;
    private static final int FR_ENCODER_ID = 2; // SHOULD BE 10
    private static final double FR_OFFSET = Units.degreesToRadians(272.5);//317.9 + 180.); //219.6 + 180.0);

    private static final int BL_DRIVE_ID = 5;
    private static final int BL_TURN_ID = 6;
    private static final int BL_ENCODER_ID = 0; // SHOULD BE 11
    private static final double BL_OFFSET = -Units.degreesToRadians(96.7);//87.7 + 180.); //135.4 + 180.0);

    private static final int BR_DRIVE_ID = 7;
    private static final int BR_TURN_ID = 8;
    private static final int BR_ENCODER_ID = 3; // SHOULD BE 12
    private static final double BR_OFFSET = Units.degreesToRadians(140. - 98.);//345.65 + 180.);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 0.0001;

    private static final int TURN_CURRENT_LIMIT = 20;
    private static final int DRIVE_SUPPLY_LIMIT = 30;
    private static final int DRIVE_STATOR_LIMIT = 80;

    private final static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private static final SwerveDrivetrainConfiguration DRIVE_CONFIG = new SwerveDrivetrainConfiguration(DRIVE_kP,
            TURN_kP,
            ALLOWED_CLOSED_LOOP_ERROR,
            TURN_CURRENT_LIMIT,
            DRIVE_SUPPLY_LIMIT,
            DRIVE_STATOR_LIMIT,
            CAN_BUS,
            FEED_FORWARD,
            CONFIGURATION);

    private static SwerveModuleConfiguration m_frontLeftConfig = new SwerveModuleConfiguration(
            FL_DRIVE_ID,
            FL_TURN_ID,
            FL_ENCODER_ID,
            FL_OFFSET,
            DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_frontRightConfig = new SwerveModuleConfiguration(
            FR_DRIVE_ID,
            FR_TURN_ID,
            FR_ENCODER_ID,
            FR_OFFSET,
            DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backLeftConfig = new SwerveModuleConfiguration(
            BL_DRIVE_ID,
            BL_TURN_ID,
            BL_ENCODER_ID,
            BL_OFFSET,
            DRIVE_CONFIG);

    private static SwerveModuleConfiguration m_backRightConfig = new SwerveModuleConfiguration(
            BR_DRIVE_ID,
            BR_TURN_ID,
            BR_ENCODER_ID,
            BR_OFFSET,
            DRIVE_CONFIG);

    public Mk2SwerveDrivetrain() {
        super(
                m_frontLeftConfig,
                m_frontRightConfig,
                m_backLeftConfig,
                m_backRightConfig,
                PHYSICS,
                m_gyro,
                PIDConstants.Theta.gains,
                AUTON_DRIVE_GAINS);
    }

    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("FL angle", Math.toDegrees(m_FL.getAbsoluteTurningEncoderRadians()));
        SmartDashboard.putNumber("FR angle", Math.toDegrees(m_FR.getAbsoluteTurningEncoderRadians()));
        SmartDashboard.putNumber("BL angle", Math.toDegrees(m_BL.getAbsoluteTurningEncoderRadians()));
        SmartDashboard.putNumber("BR angle", Math.toDegrees(m_BR.getAbsoluteTurningEncoderRadians()));

        // SmartDashboard.putNumber("FL getangle", m_FL.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FR getangle", m_FR.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BL getangle", m_BL.getState().angle.getDegrees());
        // SmartDashboard.putNumber("BR getangle", m_BR.getState().angle.getDegrees());

        m_field.setRobotPose(getPoseMeters());
        SmartDashboard.putData(m_field);

        SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());
    }

    public static Mk2SwerveDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Mk2SwerveDrivetrain();
        }
        return m_instance;
    }
}
