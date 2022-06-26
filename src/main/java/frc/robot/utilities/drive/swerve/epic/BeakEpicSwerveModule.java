// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve.epic;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.drive.swerve.SwerveModuleConfiguration;
import frc.robot.utilities.encoder.BeakAbsoluteEncoder;
import frc.robot.utilities.motor.BeakMotorController;

/** Base class for any non-differential swerve module. */
public class BeakEpicSwerveModule {
    protected double turnCPR;

    // Calculated from Drive CPR.
    protected double driveEncoderDistancePerPulse;

    protected SimpleMotorFeedforward m_feedforward;

    protected BeakMotorController m_driveMotor;
    protected BeakMotorController m_turningMotor;
    protected BeakAbsoluteEncoder m_turningEncoder;

    /**
     * Construct a new Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakEpicSwerveModule(EpicSwerveModuleConfiguration config) {}

    /**
     * Call this function in a subclass AFTER setting up motors and encoders
     */
    protected void setup(EpicSwerveModuleConfiguration config) {
        turnCPR = config.turnGearRatio * m_turningMotor.getPositionEncoderCPR();
        driveEncoderDistancePerPulse = (config.wheelDiameter * Math.PI)
                / (config.driveGearRatio * m_driveMotor.getVelocityEncoderCPR());

        m_feedforward = config.feedforward;

        configTurningEncoder(config);
        configTurningMotor(config);
        configDriveMotor(config);
    }

    public void configDriveMotor(EpicSwerveModuleConfiguration config) {
        m_driveMotor.restoreFactoryDefault();

        m_driveMotor.setBrake(true);
        m_driveMotor.setInverted(config.driveInverted);

        // Prevent the motors from drawing several hundred amps of current,
        // and allow them to run at the same speed even when voltage drops.
        m_driveMotor.setVoltageCompensationSaturation(12.0);
        m_driveMotor.setSupplyCurrentLimit(config.driveSupplyCurrentLimit);
        m_driveMotor.setStatorCurrentLimit(config.driveStatorCurrentLimit);

        // Configure PID
        m_driveMotor.setP(config.drive_kP, 0);
    }

    public void configTurningMotor(EpicSwerveModuleConfiguration config) {
        m_turningMotor.restoreFactoryDefault();

        m_turningMotor.setBrake(true);
        m_turningMotor.setInverted(config.turnInverted);

        // Initialize the encoder's position--MUST BE DONE AFTER
        // CONFIGURING TURNING ENCODER!
        resetTurningMotor();

        // Generally, turning motor current draw isn't a problem.
        // This is done to prevent stalls from killing the motor.
        m_turningMotor.setSupplyCurrentLimit(config.turnCurrentLimit);

        m_turningMotor.setAllowedClosedLoopError(config.allowedError, 0);

        m_turningMotor.setP(config.turn_kP, 0);
    }

    public void configTurningEncoder(EpicSwerveModuleConfiguration config) {
        m_turningEncoder.setAbsoluteOffset(Units.radiansToDegrees(config.angleOffset));

        // Prevent huge CAN spikes
        m_turningEncoder.setDataFramePeriod(101);
    }

    /* State Management */

    /**
     * Get the module's current state.
     * 
     * @return Current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getVelocityNU() * driveEncoderDistancePerPulse * 10,
                new Rotation2d(getTurningEncoderRadians()));
    }

    /**
     * Set the desired state for the module, and run the motors.
     * 
     * @param desiredState Desired {@link SwerveModuleState} containing the speed
     *                     and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning more than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        // Calculate Arb Feed Forward for drive motor
        // TODO: calc from SysId
        double arbFeedforward = m_feedforward.calculate(optimizedState.speedMetersPerSecond) / 12.0;

        // TODO: why divide by 10?
        // So actually it's because of the 100ms thing with Talons I think...
        m_driveMotor.setVelocityNU(
                optimizedState.speedMetersPerSecond / 10.0 / driveEncoderDistancePerPulse,
                arbFeedforward,
                0);

        // Set the turning motor to the correct position.
        setAngle(optimizedState.angle.getDegrees());
    }

    /** Encoders & Heading */

    /**
     * Set the turning motor's position to match the reported
     * angle from the CANCoder.
     */
    public void resetTurningMotor() {
        m_turningMotor.setEncoderPositionNU(
                m_turningEncoder.getAbsolutePosition() / 360.0 * turnCPR);
    }

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getTurningEncoderRadians() {
        return m_turningMotor.getPositionNU() * 2.0 * Math.PI / turnCPR;
    }

    /**
     * Zero all encoders, in case things have gone bad
     */
    public void resetEncoders() {
        m_driveMotor.setEncoderPositionNU(0);
        m_turningMotor.setEncoderPositionNU(0);
    }

    /**
     * Set the wheel's angle.
     * 
     * @param newAngle Angle to turn the wheel to, in degrees.
     */
    public void setAngle(double newAngle) {
        // Get current wheel angle in degrees
        double currentAngle = Units.radiansToDegrees(getTurningEncoderRadians());

        double remainder = Math.IEEEremainder(currentAngle, 360.0);

        // Make sure the new angle isn't too far off from the current
        // TODO: sussy, check if the reported wheel angles make sense
        double newAngleDemand = newAngle + currentAngle - remainder;

        double angleDelta = newAngleDemand - currentAngle;
        // Ensuring it stays within [-180, 180]
        // I think that because of this check it will never reach an angle above 540
        // degrees?
        if (angleDelta > 180.1) {
            newAngleDemand -= 360.0;
        } else if (angleDelta < -180.1) {
            newAngleDemand += 360.0;
        }

        m_turningMotor.setPositionNU(newAngleDemand / 360.0 * turnCPR);
    }

    public static BeakEpicSwerveModule fromSwerveModuleConfig(EpicSwerveModuleConfiguration config) {
        switch(config.moduleType) {
            case MK4i:
                return new BeakEpicMk4iSwerveModule(config);
            default:
                return null;
        }
    }
}
