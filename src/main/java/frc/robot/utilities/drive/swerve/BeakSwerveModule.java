// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

// TODO: Ambiguity
// BeakAnalogEncoder?
// TODO: Perhaps all the heavy-lifting stuff could be put here.
// This *would* require BeakAnalogEncoder, and status frame period setting on BeakMotorController
/** Add your docs here. */
public class BeakSwerveModule {
    protected BeakSwerveModule() {}

    /**
     * Construct a new generic Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakSwerveModule(SwerveModuleConfiguration config) {}

    public void configDriveMotor(SwerveModuleConfiguration config) {}

    public void configTurningMotor(SwerveModuleConfiguration config) {}

    public void configTurningEncoder(SwerveModuleConfiguration config) {}

    /* State Management */

    /**
     * Get the module's current state.
     * 
     * @return Current state of the module.
     */
    public SwerveModuleState getState() {
        return null;
    }

    /**
     * Set the desired state for the module, and run the motors.
     * 
     * @param desiredState Desired {@link SwerveModuleState} containing the speed
     *                     and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {}

    /** Encoders & Heading */

    /**
     * Set the turning motor's position to match the reported
     * angle from the CANCoder.
     */
    public void resetTurningMotor() {}

    /**
     * Get the angle of the wheel.
     * 
     * @return Angle of the wheel in radians.
     */
    public double getTurningEncoderRadians() {
        return 0;
    }

    /**
     * Zero all encoders, in case things have gone bad
     */
    public void resetEncoders() {}

    /**
     * Set the wheel's angle.
     * 
     * @param newAngle Angle to turn the wheel to, in degrees.
     */
    public void setAngle(double newAngle) {}

    public static BeakSwerveModule fromSwerveModuleConfig(SwerveModuleConfiguration config) {
        switch(config.moduleType) {
            case MK4i:
                return new BeakMk4iSwerveModule(config);
            default:
                return null;
        }
    }
}
