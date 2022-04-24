// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.sim.PhysicsSim;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonSRX m_FL, m_BL, m_FR, m_BR;

    private Gyro m_gyro;
    private DifferentialDriveOdometry m_odom;

    private Pose2d m_pose;
    Field2d field = new Field2d();

    private static Drivetrain m_instance = new Drivetrain();

    /* Config and initialization */
    public Drivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_odom = new DifferentialDriveOdometry(getRotation());

        m_FL = new WPI_TalonSRX(SubsystemConstants.DRIVE_FL);
        m_BL = new WPI_TalonSRX(SubsystemConstants.DRIVE_BL);
        m_FR = new WPI_TalonSRX(SubsystemConstants.DRIVE_FR);
        m_BR = new WPI_TalonSRX(SubsystemConstants.DRIVE_BR);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonSRX(m_FL, 0.5, 5100);
            PhysicsSim.getInstance().addTalonSRX(m_FR, 0.5, 5100);
            PhysicsSim.getInstance().addTalonSRX(m_BL, 0.5, 5100);
            PhysicsSim.getInstance().addTalonSRX(m_BR, 0.5, 5100);
        }
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
        configSensors();
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

        if (Robot.isSimulation()) {
            m_FL.config_kF(0, 0.2);
            m_BL.config_kF(0, 0.2);
            m_FR.config_kF(0, 0.2);
            m_BR.config_kF(0, 0.2);
        }
    }

    public void configNeutralMode() {
        m_FL.setNeutralMode(NeutralMode.Brake);
        m_BL.setNeutralMode(NeutralMode.Brake);
        m_FR.setNeutralMode(NeutralMode.Brake);
        m_BR.setNeutralMode(NeutralMode.Brake);
    }

    public void configInverted() {
        m_FL.setInverted(!Robot.isSimulation());
        m_BL.setInverted(!Robot.isSimulation());
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    public void configSensors() {
        m_FL.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 100);
        m_BL.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 100);
        m_FR.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 100);
        m_BR.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 100);
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
        DifferentialDriveWheelSpeeds speed = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        // System.out.println("Sneed");

        if (x != 0. || rot != 0.) {
            System.out.println(speed.rightMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE);
            System.out.println(speed.leftMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        }

        speed.desaturate(DriveConstants.MAX_VELOCITY);

        if (Robot.isSimulation()) {
            m_FL.set(ControlMode.Velocity, speed.leftMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE);
                    // DemandType.ArbitraryFeedForward,
                    // DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
            m_FR.set(ControlMode.Velocity, speed.rightMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE);
                    // DemandType.ArbitraryFeedForward,
                    // DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
        } else {
            m_FL.set(ControlMode.Velocity, speed.leftMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                    DemandType.ArbitraryFeedForward,
                    DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
            m_FR.set(ControlMode.Velocity, speed.rightMetersPerSecond / 10. / DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                    DemandType.ArbitraryFeedForward,
                    DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
        }
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                DriveConstants.ENCODER_DISTANCE_PER_PULSE / m_FL.getSelectedSensorVelocity(),
                DriveConstants.ENCODER_DISTANCE_PER_PULSE / m_FR.getSelectedSensorVelocity());
    }

    public void driveVolts(double left, double right) {
        System.out.println("Going");
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
                m_FL.getSelectedSensorVelocity() * DriveConstants.WHEEL_DIAMETER * 10.,
                m_FR.getSelectedSensorVelocity() * DriveConstants.WHEEL_DIAMETER * 10.);

        field.setRobotPose(m_pose);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("Heading", getHeading());//m_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("POSE", m_pose.getRotation().getDegrees());
    }
}
