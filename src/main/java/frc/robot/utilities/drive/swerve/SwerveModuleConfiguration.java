// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class SwerveModuleConfiguration {
    public int driveMotorID;
    public int turnMotorID;
    public int turnEncoderID;
    public double angleOffset;

    public double driveGearRatio;
    public double turnGearRatio;

    public double wheelDiameter;

    public boolean driveInverted;
    public boolean turnInverted;

    public double drive_kP;
    public double turn_kP;

    public double allowedError;

    public String CANBus;

    public SimpleMotorFeedforward feedforward;

    /**
     * Generate a new Swerve Module configuration.
     * 
     * @param driveMotorID  CAN ID of the drive motor.
     * @param turnMotorID   CAN ID of the turning motor.
     * @param turnEncoderID CAN ID of the CANCoder.
     * @param angleOffset   Offset of the CANCoder, in radians.
     * @param drive_kP      Proportional gain to use for the drive motor.
     * @param turn_kP       Proportional gain to use for the turning motor.
     * @param allowedError  Allowed error of the turning motor, in NU.
     * @param CANBus        CAN Bus that the drivetrain lies on.
     * @param feedforward   {@link SimpleMotorFeedforward} for the drivetrain.
     * @param config        {@link SdsModuleConfiguration} of the modules on your drivetrain.
     */
    public SwerveModuleConfiguration(
        int driveMotorID,
        int turnMotorID,
        int turnEncoderID,
        double angleOffset,
        double drive_kP,
        double turn_kP,
        double allowedError,
        String CANBus,
        SimpleMotorFeedforward feedforward,
        SdsModuleConfiguration config
    ) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.turnEncoderID = turnEncoderID;
        this.angleOffset = angleOffset;
        this.drive_kP = drive_kP;
        this.turn_kP = turn_kP;

        this.allowedError = allowedError;

        this.CANBus = CANBus;

        this.feedforward = feedforward;

        this.wheelDiameter = config.wheelDiameter;

        this.driveGearRatio = config.driveGearRatio;
        this.driveInverted = config.driveInverted;
        
        this.turnGearRatio = config.turnGearRatio;
        this.turnInverted = config.turnInverted;
    }
}
