// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.utilities.BeakSparkMAX;

/** Add your docs here. */
public class SixNEODrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakSparkMAX m_FL, m_BL, m_BL2, m_FR, m_BR, m_BR2;

    private static SixNEODrivetrain m_instance;

    public SixNEODrivetrain() {
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

        m_FL = new BeakSparkMAX(SubsystemConstants.DRIVE_FL);
        m_BL = new BeakSparkMAX(SubsystemConstants.DRIVE_BL);
        m_BL2 = new BeakSparkMAX(5);
        m_FR = new BeakSparkMAX(SubsystemConstants.DRIVE_FR);
        m_BR = new BeakSparkMAX(SubsystemConstants.DRIVE_BR);
        m_BR2 = new BeakSparkMAX(6);

        m_BL.follow(m_FL);
        m_BL2.follow(m_FL);
        m_BR.follow(m_FR);
        m_BR2.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_FL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BL2, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_FR, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BR, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BR2, DCMotor.getNEO(1));
        }

        sim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleNEOPerSide,
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
        double maxVel = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec);
        m_FL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FL.calculateFeedForward(1, maxVel), 0);
        m_BL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BL.calculateFeedForward(1, maxVel), 0);
        m_BL2.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BL2.calculateFeedForward(1, maxVel), 0);
        m_FR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FR.calculateFeedForward(1, maxVel), 0);
        m_BR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BR.calculateFeedForward(1, maxVel), 0);
        m_BR2.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BR2.calculateFeedForward(1, maxVel), 0);
    }

    public void configNeutralMode() {
        m_FL.setBrake(true);
        m_BL.setBrake(true);
        m_BL2.setBrake(true);
        m_FR.setBrake(true);
        m_BR.setBrake(true);
        m_BR2.setBrake(true);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_BL2.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
        m_BR2.setInverted(false);
    }

    
    public void drive(double x, double y, double rot) {
        double[] velocities = calcDesiredMotorVelocities(m_FL, x, rot);

        m_FL.setVelocityNU(velocities[0]);
        m_BL.setVelocityNU(velocities[0]);
        m_BL2.setVelocityNU(velocities[0]);
        m_FR.setVelocityNU(velocities[1]);
        m_BR.setVelocityNU(velocities[1]);
        m_BR2.setVelocityNU(velocities[1]);
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

    public void resetOdometry(Pose2d pose) {
        super.resetOdometry(pose);
        
        m_FL.resetEncoder();
        m_BL.resetEncoder();
        m_BL2.resetEncoder();
        m_FR.resetEncoder();
        m_BR.resetEncoder();
        m_BR2.resetEncoder();
    }

    public static SixNEODrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new SixNEODrivetrain();
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
