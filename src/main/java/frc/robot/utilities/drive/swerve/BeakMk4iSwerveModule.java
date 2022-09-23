// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.encoder.BeakCANCoder;
import frc.robot.utilities.motor.BeakTalonFX;

/** SDS MK4i Swerve Module. */
public class BeakMk4iSwerveModule extends BeakSwerveModule {
    PIDController m_turningPIDController;

    /**
     * Construct a new Mk4i Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakMk4iSwerveModule(SwerveModuleConfiguration config) {
        super(config);
        m_driveMotor = new BeakTalonFX(config.driveMotorID, config.CANBus);
        m_turningMotor = new BeakTalonFX(config.turnMotorID, config.CANBus);
        m_turningEncoder = new BeakCANCoder(config.turnEncoderID, config.CANBus);
        m_turningPIDController = new PIDController(config.turn_kP, 0., 0.001);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        super.setup(config);
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        super.configDriveMotor(config);

        // Prevent huge CAN spikes
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value, 19);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 19);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value, 253);
        m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    public void configTurningMotor(SwerveModuleConfiguration config) {
        super.configTurningMotor(config);

        // Prevent huge CAN spikes
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value, 19);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 19);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value, 253);
        m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    // @Override
    // public void setDesiredState(SwerveModuleState desiredState) {
    //     // Optimize the state to avoid spinning more than 90 degrees.
    //     SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

    //     // // Calculate Arb Feed Forward for drive motor
    //     // // TODO: calc from SysId
    //     // double arbFeedforward =
    //     // m_feedforward.calculate(optimizedState.speedMetersPerSecond) / 12.0;

    //     // m_driveMotor.setVelocityNU(
    //     // optimizedState.speedMetersPerSecond / 10.0 / driveEncoderDistancePerPulse,
    //     // arbFeedforward,
    //     // 0);
    //     m_driveMotor.set(optimizedState.speedMetersPerSecond / Units.feetToMeters(16.3));

    //     double turnOutput = m_turningPIDController.calculate(getTurningEncoderRadians(), desiredState.angle.getRadians());

    //     // Calculate the turning motor output from the turning PID controller.
    //     m_turningMotor.set(turnOutput);

    //     // Set the turning motor to the correct position.
    //     // setAngle(optimizedState.angle.getDegrees());
    // }
}
