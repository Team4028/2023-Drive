// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.NEODriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class FourNEODrivetrain extends BeakDifferentialDrivetrain {
    private CANSparkMax m_FL, m_BL, m_FR, m_BR;
    private RelativeEncoder m_FLEncoder, m_BLEncoder, m_FREncoder, m_BREncoder;
    private SparkMaxPIDController m_FLPid, m_BLPid, m_FRPid, m_BRPid;

    private Gyro m_gyro;
    private Pose2d m_pose;

    private static FourNEODrivetrain m_instance;
    /** Creates a new BeakNEODrivetrain. */
    public FourNEODrivetrain() {
        super(NEODriveConstants.DRIVE_KINEMATICS);

        m_gyro = new AHRS(SPI.Port.kMXP);

        m_FL = new CANSparkMax(SubsystemConstants.DRIVE_FL, MotorType.kBrushless);
        m_BL = new CANSparkMax(SubsystemConstants.DRIVE_BL, MotorType.kBrushless);
        m_FR = new CANSparkMax(SubsystemConstants.DRIVE_FR, MotorType.kBrushless);
        m_BR = new CANSparkMax(SubsystemConstants.DRIVE_BR, MotorType.kBrushless);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        m_FLEncoder = m_FL.getEncoder();
        m_BLEncoder = m_BL.getEncoder();
        m_FREncoder = m_FR.getEncoder();
        m_BREncoder = m_BR.getEncoder();

        m_FLPid = m_FL.getPIDController();
        m_BLPid = m_BL.getPIDController();
        m_FRPid = m_FR.getPIDController();
        m_BRPid = m_BR.getPIDController();
    }

    public void configMotors() {
        configPID(PIDConstants.Drive.kP, PIDConstants.Drive.kD);
        configNeutralMode();
        configInverted();
    }

    public void configPID(
        double kP,
        double kD
    ) {
        // TODO: get these from SysId
        m_FLPid.setP(kP);
        m_FLPid.setD(kD);

        m_BLPid.setP(kP);
        m_BLPid.setD(kD);

        m_FRPid.setP(kP);
        m_FRPid.setD(kD);

        m_BRPid.setP(kP);
        m_BRPid.setD(kD);
    }

    public void configNeutralMode() {
        m_FL.setIdleMode(IdleMode.kBrake);
        m_BL.setIdleMode(IdleMode.kBrake);
        m_FR.setIdleMode(IdleMode.kBrake);
        m_BR.setIdleMode(IdleMode.kBrake);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways) -- UNUSED FOR NOW
     *            NOT SWERVE.
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        DifferentialDriveWheelSpeeds speed = calcWheelSpeeds(
            x,
            rot,
            NEODriveConstants.MAX_VELOCITY,
            AutonConstants.MAX_ANGULAR_VELOCITY
        );

        //TODO: PID and testing

        m_FL.set(speed.leftMetersPerSecond / NEODriveConstants.MAX_VELOCITY);
        m_FR.set(speed.rightMetersPerSecond / NEODriveConstants.MAX_VELOCITY);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
