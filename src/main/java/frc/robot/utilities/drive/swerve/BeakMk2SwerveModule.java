// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.encoder.BeakAnalogInput;
import frc.robot.utilities.motor.BeakSparkMAX;

/** SDS Mk2 Swerve Module. */
public class BeakMk2SwerveModule extends BeakSwerveModule {
    PIDController m_turningPIDController;

    /**
     * Construct a new Mk2 Swerve Module.
     * 
     * @param config {@link SwerveModuleConfiguration} containing
     *               details of the module.
     */
    public BeakMk2SwerveModule(SwerveModuleConfiguration config) {
        super(config);
        m_driveMotor = new BeakSparkMAX(config.driveMotorID);
        m_turningMotor = new BeakSparkMAX(config.turnMotorID);
        m_turningEncoder = new BeakAnalogInput(config.turnEncoderID);

        m_turningPIDController = new PIDController(config.turn_kP, 0., 0.001);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        super.setup(config);
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        super.configDriveMotor(config);

        // Prevent huge CAN spikes
        // m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value, 19);
        // m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value,
        // 19);
        // m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value,
        // 253);
        // m_driveMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    public void configTurningMotor(SwerveModuleConfiguration config) {
        super.configTurningMotor(config);

        // Prevent huge CAN spikes
        // m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_1_General.value,
        // 19);
        // m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_2_Feedback0.value,
        // 19);
        // m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_4_AinTempVbat.value,
        // 253);
        // m_turningMotor.setStatusPeriod(StatusFrameEnhanced.Status_6_Misc.value, 59);
    }

    @Override
    public void setAngle(double angle) { // TODO: Angle motor PID
        double turnOutput = m_turningPIDController.calculate(getTurningEncoderRadians(), Math.toRadians(angle));

        // Calculate the turning motor output from the turning PID controller.
        m_turningMotor.set(turnOutput);

        SmartDashboard.putNumber("state " + bruh, m_turningMotor.getPositionNU() * 360. / turnCPR);
    }
}
