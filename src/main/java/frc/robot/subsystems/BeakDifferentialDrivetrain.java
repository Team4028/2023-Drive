// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/** Add your docs here. */
public class BeakDifferentialDrivetrain extends BeakDrivetrain {
    public DifferentialDriveKinematics m_kinematics;
    public DifferentialDriveOdometry m_odom;

    public BeakDifferentialDrivetrain(DifferentialDriveKinematics kinematics) {
        m_kinematics = kinematics;
    }

    /**
     * Calculate wheel speeds from x and rotation values from joysticks.
     * 
     * @param x Speed of the robot in the x direction (forward).
     * @param rot Angular rate of the robot.
     * @param maxVelocity Max wanted/achievable velocity of the robot, in meters per second.
     * @param maxAngularVelocity Max wanted/achievable angular velocity of the robot, in radians per second.
     */
    public DifferentialDriveWheelSpeeds calcWheelSpeeds(
        double x,
        double rot,
        double maxVelocity,
        double maxAngularVelocity
    ) {
        x *= maxVelocity;
        rot *= maxAngularVelocity;
        DifferentialDriveWheelSpeeds speed = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        speed.desaturate(maxVelocity);

        return speed;
    }

    /**
     * Get the current wheel speeds.
     * @return Current wheel speeds of the robot.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return null;
    }

    /**
     * Drive by sending voltage to the motors.
     * 
     * @param left Volts to send to the left side of the drivetrain.
     * @param right Volts to send to the right side of the drivetrain.
     */
    public void driveVolts(double left, double right) {}

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
}
