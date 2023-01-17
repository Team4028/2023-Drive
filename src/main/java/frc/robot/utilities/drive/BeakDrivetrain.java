// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utilities.subsystem.BeakGyroSubsystem;
import frc.robot.utilities.units.AngularVelocity;
import frc.robot.utilities.units.Distance;
import frc.robot.utilities.units.Velocity;

/** Base drivetrain class. */
public class BeakDrivetrain extends BeakGyroSubsystem {
    public Pose2d m_pose;

    protected Velocity m_maxVelocity;
    protected AngularVelocity m_maxAngularVelocity;

    protected Distance m_trackWidth;
    protected Distance m_wheelBase;

    protected Distance m_wheelDiameter;

    protected double m_gearRatio;

    protected SimpleMotorFeedforward m_feedForward;

    protected ProfiledPIDController m_thetaController;
    protected PIDController m_autonThetaController;
    protected PIDController m_driveController;

    /**
     * Construct a new generic drivetrain.
     * 
     * @param physics       A {@link RobotPhysics} object containing the relevant
     *                      information for your robot.
     * @param thetaPIDGains The PID gains for the theta controller.
     * @param drivePIDGains The PID gains for the auton drive controller.
     */
    public BeakDrivetrain(
            RobotPhysics physics,
            double[] thetaPIDGains,
            double[] drivePIDGains,
            boolean gyroInverted) {
        super(gyroInverted);
        m_maxVelocity = physics.maxVelocity;
        m_maxAngularVelocity = physics.maxAngularVelocity;
        m_trackWidth = physics.trackWidth;
        m_wheelBase = physics.wheelBase;
        m_wheelDiameter = physics.wheelDiameter;
        m_gearRatio = physics.driveGearRatio;
        m_feedForward = physics.feedforward;

        final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
                physics.maxAngularVelocity.getAsRadiansPerSecond(), physics.maxAngularVelocity.getAsRadiansPerSecond());

        m_thetaController = new ProfiledPIDController(
                thetaPIDGains[0],
                thetaPIDGains[1],
                thetaPIDGains[2],
                thetaConstraints);

        m_autonThetaController = new PIDController(
                thetaPIDGains[0],
                thetaPIDGains[1],
                thetaPIDGains[2]);

        m_driveController = new PIDController(
                drivePIDGains[0],
                drivePIDGains[1],
                drivePIDGains[2]);
    }

    public void configMotors() {
    }

    public RobotPhysics getPhysics() {
        return new RobotPhysics(
                m_maxVelocity,
                m_maxAngularVelocity,
                m_trackWidth,
                m_wheelBase,
                m_wheelDiameter,
                m_gearRatio,
                m_feedForward);
    }

    /**
     * Get the theta controller for auton usage.
     * 
     * @return Theta PID Controller.
     */
    public ProfiledPIDController getThetaController() {
        return m_thetaController;
    }

    /**
     * Get the drive controller for auton usage.
     *
     * @return Auton Drive PID Controller.
     */
    public PIDController getDriveController() {
        return m_driveController;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return m_feedForward;
    }

    /**
     * Gets a command to control the
     * drivetrain to follow a path.
     * 
     * @param traj Trajectory to follow.
     * @return A {@link SequentialCommandGroup} to run the trajectory, and stop the
     *         drivetrain.
     */
    public SequentialCommandGroup getTrajectoryCommand(PathPlannerTrajectory traj) {
        return null;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x             Speed of the robot in the x direction (forward).
     * @param y             Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether or not the x and y speeds are relative to
     *                      the field, for holonomic drivetrains.
     */
    public void drive(double x, double y, double rot, boolean fieldRelative) {
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        drive(x, y, rot, false);
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
     * Get the angle to a target position on the field.
     * </p>
     * 
     * Positions are relative to the bottom-left corner of the field (for Rapid
     * React, the blue alliance HP station)
     * 
     * @param x The X position of the target, in inches.
     * @param y The Y position of the target, in inches.
     * @return A {@link Rotation2d} of the drivetrain's angle to the target
     *         position.
     */
    public Rotation2d getAngleToTargetPosition(Distance x, Distance y) {
        double xDelta = x.getAsMeters() - getPoseMeters().getX();
        double yDelta = y.getAsMeters() - getPoseMeters().getY();

        double radiansToTarget = Math.atan2(yDelta, xDelta);

        return new Rotation2d(radiansToTarget);
    }
}