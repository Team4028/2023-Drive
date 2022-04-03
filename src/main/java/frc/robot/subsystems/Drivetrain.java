// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonSRX m_FL, m_BL, m_FR, m_BR;

    private AHRS m_gyro;
    private DifferentialDriveOdometry m_odom;

    private Pose2d m_pose;

    private static Drivetrain m_instance = new Drivetrain();

    /* Config and initialization */
    public Drivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_odom = new DifferentialDriveOdometry(getRotation());

        m_FL = new WPI_TalonSRX(SubsystemConstants.kFL);
        m_BL = new WPI_TalonSRX(SubsystemConstants.kBL);
        m_FR = new WPI_TalonSRX(SubsystemConstants.kFR);
        m_BR = new WPI_TalonSRX(SubsystemConstants.kBR);

        configMotors();
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        // TODO: get these from SysId
        m_FL.config_kP(0, PIDConstants.Drive.kP);
        m_FL.config_kD(0, PIDConstants.Drive.kD);

        m_BL.config_kP(0, PIDConstants.Drive.kP);
        m_BL.config_kD(0, PIDConstants.Drive.kD);

        m_FR.config_kP(0, PIDConstants.Drive.kP);
        m_FR.config_kD(0, PIDConstants.Drive.kD);

        m_BR.config_kP(0, PIDConstants.Drive.kP);
        m_BR.config_kD(0, PIDConstants.Drive.kD);
    }

    public void configNeutralMode() {
        m_FL.setNeutralMode(NeutralMode.Brake);
        m_BL.setNeutralMode(NeutralMode.Brake);
        m_FR.setNeutralMode(NeutralMode.Brake);
        m_BR.setNeutralMode(NeutralMode.Brake);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    /* Drive & Auton commands */

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways) -- UNUSED FOR NOW
     *            NOT SWERVE.
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        DifferentialDriveWheelSpeeds speed = DriveConstants.kDriveKinematics.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        speed.desaturate(DriveConstants.kMaxSpeed);

        m_FL.set(ControlMode.Velocity, speed.leftMetersPerSecond / 10. / DriveConstants.kEncoderDistancePerPulse,
                DemandType.ArbitraryFeedForward,
                DriveConstants.kFF.calculate(speed.leftMetersPerSecond) / DriveConstants.kNominalVoltage);
        m_FR.set(ControlMode.Velocity, speed.rightMetersPerSecond / 10. / DriveConstants.kEncoderDistancePerPulse,
                DemandType.ArbitraryFeedForward,
                DriveConstants.kFF.calculate(speed.rightMetersPerSecond) / DriveConstants.kNominalVoltage);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                DriveConstants.kEncoderDistancePerPulse / m_FL.getSelectedSensorVelocity(),
                DriveConstants.kEncoderDistancePerPulse / m_FR.getSelectedSensorVelocity());
    }

    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_FR.setVoltage(right);
    }

    /**
     * Class for managing and driving a 4-motor
     * differential drivetrain, with 4 TalonSRX
     * controllers and a NavX.
     **/
    public static Drivetrain getInstance() {
        return m_instance;
    }

    /* NavX wrapper methods */
    public Rotation2d getRotation() {
        return m_gyro.getRotation2d();
    }

    public void zero() {
        m_gyro.reset();
    }

    public double getHeading() {
        return getRotation().getDegrees();
    }

    public double getRate() {
        return m_gyro.getRate();
    }

    /* Pose & Odometry */
    public Pose2d getPoseMeters() {
        return m_odom.getPoseMeters();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odom.resetPosition(pose, getRotation());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Rotation2d rot = getRotation();

        m_pose = m_odom.update(rot,
                m_FL.getSelectedSensorVelocity() * DriveConstants.kWheelDiameter * 10.,
                m_FR.getSelectedSensorVelocity() * DriveConstants.kWheelDiameter * 10.);

        SmartDashboard.putNumber("Heading", m_pose.getRotation().getDegrees());
    }
}
