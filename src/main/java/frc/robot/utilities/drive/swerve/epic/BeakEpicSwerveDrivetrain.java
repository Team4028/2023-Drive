// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve.epic;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.RobotPhysics;

/** Generic Swerve Drivetrain subsystem. */
public class BeakEpicSwerveDrivetrain extends BeakDrivetrain {
    List<BeakEpicSwerveModule> m_modules;
    int m_numModules;

    protected SwerveDriveOdometry m_odom;
    protected SwerveDriveKinematics m_kinematics;

    protected RobotPhysics m_physics;

    /**
     * Create a new Swerve drivetrain.
     * 
     * @param physics       {@link RobotPhysics} containing the robot's physical
     *                      details.
     * @param gyro          The gyroscope used by this drivetrain.
     * @param thetaPIDGains The PID gains for the theta controller.
     * @param drivePIDGains The PID gains for the auton drive controller.
     * @param configs       Configurations for all swerve modules.
     */
    public BeakEpicSwerveDrivetrain(
            RobotPhysics physics,
            Gyro gyro,
            double[] thetaPIDGains,
            double[] drivePIDGains,
            EpicSwerveModuleConfiguration... configs) {
        super(physics,
                thetaPIDGains,
                drivePIDGains);

        m_physics = physics;

        m_numModules = configs.length;
        Translation2d[] moduleLocations = {};

        for (int i = 0; i < m_numModules; i++) {
            m_modules.add(BeakEpicSwerveModule.fromSwerveModuleConfig(configs[i]));
            moduleLocations[i] = configs[i].moduleLocation;
        }

        m_gyro = gyro;

        m_kinematics = new SwerveDriveKinematics(moduleLocations);
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
                getModuleStates());

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

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d())
                        : new ChassisSpeeds(x, y, rot));

        setModuleStates(states);
    }

    /* Swerve-specific Methods */

    /**
     * Set each module's {@link SwerveModuleState}.
     * 
     * @param desiredStates An array of the desired states for the m_modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_physics.maxVelocity);

        for (int i = 0; i < desiredStates.length; i++) {
            m_modules.get(i).setDesiredState(desiredStates[i]);
        }
    }

    /**
     * Get the states of each module.
     * 
     * @return Array of {@link SwerveModuleState}s for each module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {};
        for (int i = 0; i < m_numModules; i++) {
            states[i] = m_modules.get(i).getState();
        }

        return states;
    }

    /**
     * Reset all drive and turning encoders to zero.
     */
    public void resetEncoders() {
        for (BeakEpicSwerveModule module : m_modules) {
            module.resetEncoders();
        }
    }

    /**
     * Re-zero all turning encoders to match the CANCoder.
     */
    public void resetTurningMotors() {
        for (BeakEpicSwerveModule module : m_modules) {
            module.resetTurningMotor();
        }
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
