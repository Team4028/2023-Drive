// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Robot;
import frc.robot.utilities.BeakMotorController;
import frc.robot.utilities.Util;

/** Add your docs here. */
public class BeakDifferentialDrivetrain extends BeakDrivetrain {
    public DifferentialDriveKinematics m_kinematics;
    public DifferentialDriveOdometry m_odom;

    protected AnalogGyroSim m_gyroSim;
    protected DifferentialDrivetrainSim sim;

    protected double encoderDistancePerPulse;

    // Additional followers (e.g. 6-NEO) must be created and configured separately
    protected BeakMotorController m_FL, m_BL, m_FR, m_BR;

    /**
     * Construct a new differential drivetrain.
     * @param maxVelocity Maximum velocity, in meters per second.
     * @param maxAngularVelocity Maximum angular (rotational) velocity, in radians per second.
     * @param trackWidth Track width (left wheel to right wheel), in inches.
     * @param wheelBase Wheel base (back wheel to front wheel), in inches
     * @param wheelDiameter Wheel diameter, in meters.
     * @param gearRatio Gear ratio of the motors.
     * @param feedForward A {@link SimpleMotorFeedforward} calculated from SysID.
     */
    public BeakDifferentialDrivetrain(
        double maxVelocity,
        double maxAngularVelocity,
        double trackWidth,
        double wheelBase,
        double wheelDiameter,
        double gearRatio,
        SimpleMotorFeedforward feedForward
    ) {
        super(maxVelocity, maxAngularVelocity, trackWidth, wheelBase, wheelDiameter, gearRatio, feedForward);
        m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(m_trackWidth));
    }

    /**
     * Calculate wheel speeds from x and rotation values from joysticks.
     * 
     * @param x Speed of the robot in the x direction (forward).
     * @param rot Angular rate of the robot.
     */
    public DifferentialDriveWheelSpeeds calcWheelSpeeds(
        double x,
        double rot
    ) {
        x *= m_maxVelocity;
        rot *= m_maxAngularVelocity;
        DifferentialDriveWheelSpeeds speed = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        speed.desaturate(m_maxVelocity);

        return speed;
    }

/**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways) -- UNUSED FOR NOW
     *            NOT SWERVE.
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        DifferentialDriveWheelSpeeds speed = calcWheelSpeeds(x, rot);

        // Assumes all motor controllers are of the same type.
        encoderDistancePerPulse = (m_wheelDiameter * Math.PI) / m_FL.getVelocityEncoderCPR();

        double rightVel = speed.rightMetersPerSecond / encoderDistancePerPulse;
        double leftVel = speed.leftMetersPerSecond / encoderDistancePerPulse;

        if (x != 0. || rot != 0.) {
            if (Robot.isSimulation()) {
                m_FR.setVelocityNU(rightVel);
                // DemandType.ArbitraryFeedForward,
                // DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) /
                // DriveConstants.NOMINAL_VOLTAGE);
                m_FL.setVelocityNU(leftVel);
                // DemandType.ArbitraryFeedForward,
                // DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) /
                // DriveConstants.NOMINAL_VOLTAGE);
            } else {
                // m_FL.set(ControlMode.Velocity, leftVel,
                //         DemandType.ArbitraryFeedForward,
                //         DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
                // m_FR.set(ControlMode.Velocity, rightVel,
                //         DemandType.ArbitraryFeedForward,
                //         DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
            }
        } else {
            m_FL.setVelocityNU(0.);
            m_FR.setVelocityNU(0.);
        }
    }

    /**
     * Get the current wheel speeds.
     * @return Current wheel speeds of the robot.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            Util.NUtoMeters(m_FL.getVelocityNU(), m_FL.getVelocityEncoderCPR(), m_gearRatio, Units.metersToInches(m_wheelDiameter)),
            Util.NUtoMeters(m_FR.getVelocityNU(), m_FR.getVelocityEncoderCPR(), m_gearRatio, Units.metersToInches(m_wheelDiameter))
        );
    }

    /**
     * Drive by sending voltage to the motors.
     * 
     * @param left Volts to send to the left side of the drivetrain.
     * @param right Volts to send to the right side of the drivetrain.
     */
    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_FR.setVoltage(right);
    }
    
    public Rotation2d getGyroRotation2d() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        } else {
            return m_gyro.getRotation2d();
        }
    }

    public double getGyroRate() {
        if (Robot.isSimulation()) {
            return m_gyroSim.getRate();
        } else {
            return m_gyro.getRate();
        }
    }

    /**
     * Get the robot's pose.
     * @return The pose reported from the odometry, measured in meters.
     */
    public Pose2d getPoseMeters() {
        return m_odom.getPoseMeters();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odom.resetPosition(pose, getGyroRotation2d());
    }
    
    public Pose2d updateOdometry() {
        if (Robot.isSimulation()) {
            sim.setInputs(
                m_FR.getOutputVoltage(),
                m_FL.getOutputVoltage());
            sim.update(0.02);

            m_gyroSim.setAngle(-sim.getHeading().getDegrees());
        }
        Rotation2d rot = getGyroRotation2d();

        m_pose = m_odom.update(rot,
            Util.NUtoMeters(m_FL.getPositionNU(), m_FL.getPositionEncoderCPR(), m_gearRatio, Units.metersToInches(m_wheelDiameter)),
            Util.NUtoMeters(m_FR.getPositionNU(), m_FR.getPositionEncoderCPR(), m_gearRatio, Units.metersToInches(m_wheelDiameter))
        );

        return m_pose;
    }
}