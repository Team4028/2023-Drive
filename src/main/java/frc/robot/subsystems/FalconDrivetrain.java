// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.utilities.BeakTalonFX;

/** Add your docs here. */
public class FalconDrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakTalonFX m_FL, m_BL, m_FR, m_BR;

    private static FalconDrivetrain m_instance;

    public FalconDrivetrain() {
        super(
                DriveConstants.MAX_VELOCITY,
                AutonConstants.MAX_ANGULAR_VELOCITY,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                DriveConstants.WHEEL_DIAMETER,
                DriveConstants.GEAR_RATIO,
                DriveConstants.FEED_FORWARD);

        m_gyro = new AHRS(SPI.Port.kMXP);
        if (Robot.isSimulation()) {
            m_gyroSim = new AnalogGyroSim(0);
        }

        m_odom = new DifferentialDriveOdometry(getGyroRotation2d());

        m_FL = new BeakTalonFX(SubsystemConstants.DRIVE_FL);
        m_BL = new BeakTalonFX(SubsystemConstants.DRIVE_BL);
        m_FR = new BeakTalonFX(SubsystemConstants.DRIVE_FR);
        m_BR = new BeakTalonFX(SubsystemConstants.DRIVE_BR);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            DCMotor falcon = DCMotor.getFalcon500(1);
            double accel = 0.5;
            double maxVel = Units.radiansPerSecondToRotationsPerMinute(falcon.freeSpeedRadPerSec) * 2048 / 600;
            PhysicsSim.getInstance().addTalonFX(m_FL, accel, maxVel);
            PhysicsSim.getInstance().addTalonFX(m_FR, accel, maxVel);
            PhysicsSim.getInstance().addTalonFX(m_BL, accel, maxVel);
            PhysicsSim.getInstance().addTalonFX(m_BR, accel, maxVel);
        }

        sim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleFalcon500PerSide,
                KitbotGearing.k5p95,
                KitbotWheelSize.kSixInch,
                null);
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        // TODO: get these from SysId
        double maxVel = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec) * 2048
                / 600;

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
        m_BL.setVelocityNU(velocities[0]);
        m_FR.setVelocityNU(velocities[1]);
        m_BR.setVelocityNU(velocities[1]);
    }

    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_BL.setVoltage(left);
        m_FR.setVoltage(right);
        m_BR.setVoltage(right);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return super.getWheelSpeeds(m_FL, m_FR);
    }

    public static FalconDrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new FalconDrivetrain();
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
