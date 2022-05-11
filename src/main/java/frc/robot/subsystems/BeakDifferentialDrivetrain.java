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

    /**
     * Construct a new differential drivetrain.
     * 
     * @param maxVelocity        Maximum velocity, in meters per second.
     * @param maxAngularVelocity Maximum angular (rotational) velocity, in radians
     *                           per second.
     * @param trackWidth         Track width (left wheel to right wheel), in inches.
     * @param wheelBase          Wheel base (back wheel to front wheel), in inches
     * @param wheelDiameter      Wheel diameter, in meters.
     * @param gearRatio          Gear ratio of the motors.
     * @param feedForward        A {@link SimpleMotorFeedforward} calculated from
     *                           SysID.
     */
    public BeakDifferentialDrivetrain(
            double maxVelocity,
            double maxAngularVelocity,
            double trackWidth,
            double wheelBase,
            double wheelDiameter,
            double gearRatio,
            SimpleMotorFeedforward feedForward) {
        super(maxVelocity, maxAngularVelocity, trackWidth, wheelBase, wheelDiameter, gearRatio, feedForward);
        m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(m_trackWidth));
    }

    /**
     * Calculate wheel speeds from x and rotation values from joysticks.
     * 
     * @param x   Speed of the robot in the x direction (forward).
     * @param rot Angular rate of the robot.
     */
    public DifferentialDriveWheelSpeeds calcWheelSpeeds(
            double x,
            double rot) {
        x *= m_maxVelocity;
        rot *= m_maxAngularVelocity;
        DifferentialDriveWheelSpeeds speed = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        speed.desaturate(m_maxVelocity);

        return speed;
    }

    /**
     * Method to calculate needed motor velocities for the left and right side of the drivetrain.</p>
     * 
     * x and rot values should be from joysticks.
     * @param motorController Any motor controller on the drivetrain.
     * @param x Speed of the robot in the x direction (forward).
     * @param rot Angular rate of the robot.
     * @return {left velocity in NU, right velocity in NU}
     */
    public double[] calcDesiredMotorVelocities(
        BeakMotorController motorController,
        double x,
        double rot
    ) {
        DifferentialDriveWheelSpeeds speed = calcWheelSpeeds(x, rot);

        // Assumes all motor controllers are of the same type.
        encoderDistancePerPulse = (Units.inchesToMeters(m_wheelDiameter) * Math.PI) / motorController.getVelocityEncoderCPR();

        double rightVel = speed.rightMetersPerSecond / encoderDistancePerPulse;
        double leftVel = speed.leftMetersPerSecond / encoderDistancePerPulse;

        double[] vels = {leftVel, rightVel};

        return vels;
    }

    /**
     * Get the current wheel speeds.
     * 
     * @return Current wheel speeds of the robot.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return null;
    }

    /**
     * Get the current wheel speeds.
     * 
     * @return Current wheel speeds of the robot.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(BeakMotorController frontLeft, BeakMotorController frontRight) {
        return new DifferentialDriveWheelSpeeds(
                Util.NUtoMeters(frontLeft.getVelocityNU(), frontLeft.getVelocityEncoderCPR(), m_gearRatio,
                        m_wheelDiameter),
                Util.NUtoMeters(frontRight.getVelocityNU(), frontRight.getVelocityEncoderCPR(), m_gearRatio,
                        m_wheelDiameter));
    }

    /**
     * Drive by sending voltage to the motors.
     * 
     * @param left  Volts to send to the left side of the drivetrain.
     * @param right Volts to send to the right side of the drivetrain.
     */
    public void driveVolts(double left, double right) {}

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
     * 
     * @return The pose reported from the odometry, measured in meters.
     */
    public Pose2d getPoseMeters() {
        return m_odom.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odom.resetPosition(pose, getGyroRotation2d());
    }

    public Pose2d updateOdometry(
        BeakMotorController frontLeftMotorController,
        BeakMotorController frontRightMotorController
    ) {
        if (Robot.isSimulation()) {
            sim.setInputs(
                    frontRightMotorController.getOutputVoltage(),
                    frontLeftMotorController.getOutputVoltage());
            sim.update(0.02);

            m_gyroSim.setAngle(-sim.getHeading().getDegrees());
        }
        Rotation2d rot = getGyroRotation2d();

        m_pose = m_odom.update(rot,
                Util.NUtoMeters(frontLeftMotorController.getPositionNU(), frontLeftMotorController.getPositionEncoderCPR(), m_gearRatio,
                        m_wheelDiameter),
                Util.NUtoMeters(frontRightMotorController.getPositionNU(), frontRightMotorController.getPositionEncoderCPR(), m_gearRatio,
                        m_wheelDiameter));

        return m_pose;
    }

    // FIXME: Some math is off here somehow
    public Rotation2d getAngleToTargetPosition(double x, double y) {
        double xDelta = Units.inchesToMeters(x) - m_odom.getPoseMeters().getX();
        double yDelta = Units.inchesToMeters(y) - m_odom.getPoseMeters().getY();

        double radiansToTarget = Math.atan2(xDelta, yDelta);

        System.out.println(Units.radiansToDegrees(radiansToTarget));
        return new Rotation2d(radiansToTarget);
    }
}