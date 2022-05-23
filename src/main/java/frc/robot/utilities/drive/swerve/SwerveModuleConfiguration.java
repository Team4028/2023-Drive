// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Class containing general configuration for a {@link BeakSwerveModule}. */
public class SwerveModuleConfiguration {
    public enum ModuleType {
        MK2,
        MK3,
        MK4,
        MK4i
    }

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

    public int turnCurrentLimit;

    public int driveSupplyCurrentLimit;
    public int driveStatorCurrentLimit;

    public String CANBus;

    public SimpleMotorFeedforward feedforward;

    public ModuleType moduleType;

    /**
     * Generate a new Swerve Module configuration.
     * 
     * @param driveMotorID      CAN ID of the drive motor.
     * @param turnMotorID       CAN ID of the turning motor.
     * @param turnEncoderID     CAN ID of the CANCoder.
     * @param angleOffset       Offset of the CANCoder, in radians.
     * @param drive_kP          Proportional gain to use for the drive motor.
     * @param turn_kP           Proportional gain to use for the turning motor.
     * @param allowedError      Allowed error of the turning motor, in NU.
     * @param turnCurrentLimit  Current limit of the turning motor.
     * @param driveSupplyLimit  Supply current limit of the drive motor.
     * @param driveStatorLimit  Stator current limit of the drive motor (if on Falcons).
     * @param CANBus            CAN Bus that the drivetrain lies on.
     * @param feedforward       {@link SimpleMotorFeedforward} for the drivetrain.
     * @param config            {@link SdsModuleConfiguration} of the modules on your
     *                          drivetrain.
     * @param ModuleType        The type of the module, i.e. MK2, MK3, etc.
     */
    public SwerveModuleConfiguration(
            int driveMotorID,
            int turnMotorID,
            int turnEncoderID,
            double angleOffset,
            double drive_kP,
            double turn_kP,
            double allowedError,
            int turnCurrentLimit,
            int driveSupplyLimit,
            int driveStatorLimit,
            String CANBus,
            SimpleMotorFeedforward feedforward,
            SdsModuleConfiguration config,
            ModuleType moduleType) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.turnEncoderID = turnEncoderID;
        this.angleOffset = angleOffset;
        this.drive_kP = drive_kP;
        this.turn_kP = turn_kP;

        this.allowedError = allowedError;

        this.turnCurrentLimit = turnCurrentLimit;
        this.driveSupplyCurrentLimit = driveSupplyLimit;
        this.driveStatorCurrentLimit = driveStatorLimit;

        this.CANBus = CANBus;

        this.feedforward = feedforward;

        this.wheelDiameter = config.wheelDiameter;

        this.driveGearRatio = config.driveGearRatio;
        this.driveInverted = config.driveInverted;

        this.turnGearRatio = config.turnGearRatio;
        this.turnInverted = config.turnInverted;

        this.moduleType = moduleType;
    }
}
