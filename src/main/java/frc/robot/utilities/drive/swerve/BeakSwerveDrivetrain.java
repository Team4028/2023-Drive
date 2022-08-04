// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.RobotPhysics;

/** Generic Swerve Drivetrain subsystem. */
public class BeakSwerveDrivetrain extends BeakDrivetrain {
    protected BeakSwerveModule m_FL;
    protected BeakSwerveModule m_FR;
    protected BeakSwerveModule m_BL;
    protected BeakSwerveModule m_BR;

    protected SwerveDriveOdometry m_odom;
    protected SwerveDriveKinematics m_kinematics;

    protected RobotPhysics m_physics;

    /**
     * Create a new Swerve Drivetrain.
     * 
     * @param frontLeftConfig  {@link SwerveModuleConfiguration} for the front left
     *                         module.
     * @param frontRightConfig {@link SwerveModuleConfiguration} for the front right
     *                         module.
     * @param backLeftConfig   {@link SwerveModuleConfiguration} for the back left
     *                         module.
     * @param backRightConfig  {@link SwerveModuleConfiguration} for the back right
     *                         module.
     * @param physics          {@link RobotPhysics} containing the robot's physical
     *                         details.
     * @param pigeonID         ID of the Pigeon2 IMU.
     */
    public BeakSwerveDrivetrain(
            SwerveModuleConfiguration frontLeftConfig,
            SwerveModuleConfiguration frontRightConfig,
            SwerveModuleConfiguration backLeftConfig,
            SwerveModuleConfiguration backRightConfig,
            RobotPhysics physics,
            Gyro gyro,
            double[] thetaPIDGains,
            double[] drivePIDGains) {
        super(physics,
                thetaPIDGains,
                drivePIDGains);

        m_physics = physics;

        m_FL = BeakSwerveModule.fromSwerveModuleConfig(frontLeftConfig);
        m_FR = BeakSwerveModule.fromSwerveModuleConfig(frontRightConfig);
        m_BL = BeakSwerveModule.fromSwerveModuleConfig(backLeftConfig);
        m_BR = BeakSwerveModule.fromSwerveModuleConfig(backRightConfig);

        m_gyro = gyro;

        m_kinematics = new SwerveDriveKinematics(
                new Translation2d(Units.inchesToMeters(physics.wheelBase)/ 2, Units.inchesToMeters(physics.trackWidth) / 2),
                new Translation2d(Units.inchesToMeters(physics.wheelBase)/ 2, -Units.inchesToMeters(physics.trackWidth) / 2),
                new Translation2d(-Units.inchesToMeters(physics.wheelBase) / 2, Units.inchesToMeters(physics.trackWidth) / 2),
                new Translation2d(-Units.inchesToMeters(physics.wheelBase) / 2, -Units.inchesToMeters(physics.trackWidth) / 2));
        
        m_odom = new SwerveDriveOdometry(m_kinematics,
        getGyroRotation2d());
    }

    public SequentialCommandGroup getTrajectoryCommand(Trajectory traj) {
        return new SwerveControllerCommand(
                traj,
                this::getPoseMeters,
                m_kinematics,
                m_driveController,
                m_driveController,
                m_thetaController,
                this::setModuleStates,
                this)
                        .andThen(() -> drive(0, 0, 0));
    }

    public Pose2d updateOdometry() {
        m_pose = m_odom.update(
                getGyroRotation2d(),
                m_FL.getState(),
                m_BL.getState(),
                m_FR.getState(),
                m_BR.getState());

        return m_pose;
    }

    public Pose2d getPoseMeters() {
        return m_odom.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odom.resetPosition(pose, getGyroRotation2d());
    }

    public void drive(double x, double y, double rot, boolean fieldRelative) {
        x *= m_physics.maxVelocity;
        y *= m_physics.maxVelocity;
        rot *= m_physics.maxAngularVelocity;

        System.out.println(rot);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, m_odom.getPoseMeters().getRotation())
                        : new ChassisSpeeds(x, y, rot));

        setModuleStates(states);
    }

    /* Swerve-specific Methods */

    /**
     * Set each module's {@link SwerveModuleState}.
     * 
     * @param desiredStates An array of the desired states for the modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_physics.maxVelocity);

        m_FL.setDesiredState(desiredStates[0]);
        m_FR.setDesiredState(desiredStates[1]);
        m_BL.setDesiredState(desiredStates[2]);
        m_BR.setDesiredState(desiredStates[3]);
    }

    /**
     * Get the states of each module.
     * 
     * @return Array of {@link SwerveModuleState}s for each module.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                m_FL.getState(),
                m_FR.getState(),
                m_BL.getState(),
                m_BR.getState()
        };
    }

    /**
     * Reset all drive and turning encoders to zero.
     */
    public void resetEncoders() {
        m_FL.resetEncoders();
        m_FR.resetEncoders();
        m_BL.resetEncoders();
        m_BR.resetEncoders();
    }

    /**
     * Re-zero all turning encoders to match the CANCoder.
     */
    public void resetTurningMotors() {
        m_FL.resetTurningMotor();
        m_FR.resetTurningMotor();
        m_BL.resetTurningMotor();
        m_BR.resetTurningMotor();
    }

    /**
     * Zero the pose and heading of the robot.
     */
    public void zero() {
        resetTurningMotors();
        resetOdometry(new Pose2d());
    }

    private ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(
                getModuleStates());
    }

    /**
     * Get the forward velocity of the drivetrain.
     * 
     * @return The X (forward) velocity of the drivetrain, in meters per second.
     */
    public double getForwardVelocity() {
        return getChassisSpeeds().vxMetersPerSecond;
    }

    /**
     * Get the sideways velocity of the drivetrain.
     * 
     * @return The Y (sideways) velocity of the drivetrain, in meters per second.
     */
    public double getSidewaysVelocity() {
        return getChassisSpeeds().vyMetersPerSecond;
    }

    /**
     * Get the angular velocity of the drivetrain.
     * 
     * @return The angular velocity of the drivetrain, in radians per second.
     */
    public double getAngularVelocity() {
        return getChassisSpeeds().omegaRadiansPerSecond;
    }
}
