// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class BeakDrivetrain extends SubsystemBase {
    public Pose2d m_pose;
    public Gyro m_gyro;

    protected double m_maxVelocity;
    protected double m_maxAngularVelocity;

    protected double m_trackWidth;
    protected double m_wheelBase;

    protected double m_wheelDiameter;

    protected double m_gearRatio;

    protected SimpleMotorFeedforward m_feedForward;

    /**
     * Construct a new generic drivetrain.
     * 
     * @param maxVelocity        Maximum velocity, in meters per second.
     * @param maxAngularVelocity Maximum angular (rotational) velocity, in radians
     *                           per second.
     * @param trackWidth         Track width (left wheel to right wheel), in inches.
     * @param wheelBase          Wheel base (back wheel to front wheel), in inches
     * @param wheelDiameter      Wheel diameter, in inches.
     * @param gearRatio          Gear ratio of the motors.
     * @param feedForward        A {@link SimpleMotorFeedforward} calculated from
     *                           SysID.
     */
    public BeakDrivetrain(
            double maxVelocity,
            double maxAngularVelocity,
            double trackWidth,
            double wheelBase,
            double wheelDiameter,
            double gearRatio,
            SimpleMotorFeedforward feedForward) {
        m_maxVelocity = maxVelocity;
        m_maxAngularVelocity = maxAngularVelocity;
        m_trackWidth = trackWidth;
        m_wheelBase = wheelBase;
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = gearRatio;
        m_feedForward = feedForward;
    }

    public void configMotors() {
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
    }

    /**
     * Gets the gyro's reported angle.
     * 
     * @return A {@link Rotation2d} containing the reported angle of the gyro.
     */
    public Rotation2d getGyroRotation2d() {
        return m_gyro.getRotation2d();
    }

    /**
     * Zero the gyro.
     */
    public void zero() {
        m_gyro.reset();
    }

    /**
     * Get the gyro's reported heading.
     * 
     * @return The reported angle of the gyro in degrees.
     */
    public double getGyroHeading() {
        return getGyroRotation2d().getDegrees();
    }

    /**
     * Get the gyro's reported rate
     * 
     * @return The reported rate of the gyro in degrees per second.
     */
    public double getGyroRate() {
        return m_gyro.getRate();
    }

    /**
     * Get the robot's pose.
     * 
     * @return The pose reported from the odometry, measured in meters.
     */
    public Pose2d getPoseMeters() {
        return null;
    }

    /**
     * Get the robot's rotation.
     * 
     * @return A {@link Rotation2d} of the reported robot rotation from the
     *         odometry.
     */
    public Rotation2d getRotation2d() {
        return getPoseMeters().getRotation();
    }

    /**
     * Get the robot's heading.
     * 
     * @return The heading reported from the odometry, in degrees.
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
    }

    /**
     * Update the robot odometry.
     * 
     * @return Updated pose.
     */
    public Pose2d updateOdometry() {
        return m_pose;
    }

    /**
     * Get the angle to a target position on the field.</p>
     * 
     * Positions are relative to the bottom-left corner of the field (for Rapid React, the blue alliance HP station)
     * @param x The X position of the target, in inches.
     * @param y The Y position of the target, in inches.
     * @return A {@link Rotation2d} of the drivetrain's angle to the target position.
     */
    public Rotation2d getAngleToTargetPosition(double x, double y) {
        double xDelta = Units.inchesToMeters(x) - getPoseMeters().getX();
        double yDelta = Units.inchesToMeters(y) - getPoseMeters().getY();

        double radiansToTarget = Math.atan2(yDelta, xDelta);

        return new Rotation2d(radiansToTarget);
    }
}