// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.sim.CTREPhysicsSim;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.motor.BeakTalonSRX;

/** Add your docs here. */
public class CIMDrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakTalonSRX m_FL, m_BL, m_FR, m_BR;

    private static CIMDrivetrain m_instance;

    public CIMDrivetrain() {
        super(
            DriveConstants.PHYSICS,
            DriveConstants.FEED_FORWARD
        );

        m_gyro = new AHRS(SPI.Port.kMXP);
        if (Robot.isSimulation()) {
            m_gyroSim = new AnalogGyroSim(0);
        }

        m_odom = new DifferentialDriveOdometry(getGyroRotation2d());

        m_FL = new BeakTalonSRX(SubsystemConstants.DRIVE_FL);
        m_BL = new BeakTalonSRX(SubsystemConstants.DRIVE_BL);
        m_FR = new BeakTalonSRX(SubsystemConstants.DRIVE_FR);
        m_BR = new BeakTalonSRX(SubsystemConstants.DRIVE_BR);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            DCMotor cim = DCMotor.getCIM(1);
            double accel = 0.5;
            double maxVel = Units.radiansPerSecondToRotationsPerMinute(cim.freeSpeedRadPerSec) * 4096 / 600;
            CTREPhysicsSim.getInstance().addTalonSRX(m_FL, accel, maxVel);
            CTREPhysicsSim.getInstance().addTalonSRX(m_FR, accel, maxVel);
            CTREPhysicsSim.getInstance().addTalonSRX(m_BL, accel, maxVel);
            CTREPhysicsSim.getInstance().addTalonSRX(m_BR, accel, maxVel);
        }

        sim = new DifferentialDrivetrainSim(
            DCMotor.getCIM(2),
            7.51,
            0.9,
            Units.lbsToKilograms(60.),
            Units.inchesToMeters(3.),
            Units.inchesToMeters(DriveConstants.TRACK_WIDTH),
            null);
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        // TODO: get these from SysId
        double maxVel = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getCIM(1).freeSpeedRadPerSec) * 4096 / 600;

        m_FL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FL.calculateFeedForward(1, maxVel), 0);
        m_BL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BL.calculateFeedForward(1, maxVel), 0);
        m_FR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FR.calculateFeedForward(1, maxVel), 0);
        m_BR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BR.calculateFeedForward(1, maxVel), 0);
    }

    public void configNeutralMode() {
        m_FL.setBrake(true);
        m_BL.setBrake(true);
        m_FR.setBrake(true);
        m_BR.setBrake(true);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    
    public void drive(double x, double y, double rot) {
        double[] velocities = calcDesiredMotorVelocities(m_FL, x, rot);

        m_FL.setVelocityNU(velocities[0]);
        m_FR.setVelocityNU(velocities[1]);
    }

    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_FR.setVoltage(right);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return super.getWheelSpeeds(m_FL, m_FR);
    }

    public void resetOdometry(Pose2d pose) {
        super.resetOdometry(pose);
        
        m_FL.resetEncoder();
        m_BL.resetEncoder();
        m_FR.resetEncoder();
        m_BR.resetEncoder();
    }

    public static CIMDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new CIMDrivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry(m_FL, m_FR);

        field.setRobotPose(m_pose);
        SmartDashboard.putData(field);
    }
}
