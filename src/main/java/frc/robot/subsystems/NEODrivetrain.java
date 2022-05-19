// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.Constants.PIDConstants;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.drive.RobotPhysics;
import frc.robot.utilities.motor.BeakSparkMAX;

/** Add your docs here. */
public class NEODrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakSparkMAX m_FL, m_BL, m_FR, m_BR;

    private static final double kP = 0.01;
    private static final double kD = 0.02;

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int FR_ID = 3;
    private static final int BR_ID = 4;

    private static final double MAX_VELOCITY = Units.feetToMeters(15.5);

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 26;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 28;

    private static final double WHEEL_DIAMETER = 6.258;
    private static final double GEAR_RATIO = 7.5;

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            0,
            0,
            0);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            0,
            TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER,
            GEAR_RATIO,
            FEED_FORWARD);

    private static NEODrivetrain m_instance;

    public NEODrivetrain() {
        super(
                PHYSICS,
                PIDConstants.Theta.gains);

        m_gyro = new AHRS(SPI.Port.kMXP);
        if (Robot.isSimulation()) {
            m_gyroSim = new AnalogGyroSim(0);
        }

        m_odom = new DifferentialDriveOdometry(getGyroRotation2d());

        m_FL = new BeakSparkMAX(FL_ID);
        m_BL = new BeakSparkMAX(BL_ID);
        m_FR = new BeakSparkMAX(FR_ID);
        m_BR = new BeakSparkMAX(BR_ID);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_FL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_FR, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BR, DCMotor.getNEO(1));
        }

        sim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                7.51,
                0.9,
                Units.lbsToKilograms(60.),
                Units.inchesToMeters(3.),
                Units.inchesToMeters(TRACK_WIDTH),
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
        m_FL.setPIDF(kP, 0., kD, m_FL.calculateFeedForward(1, maxVel), 0);
        m_BL.setPIDF(kP, 0., kD, m_BL.calculateFeedForward(1, maxVel), 0);
        m_FR.setPIDF(kP, 0., kD, m_FR.calculateFeedForward(1, maxVel), 0);
        m_BR.setPIDF(kP, 0., kD, m_BR.calculateFeedForward(1, maxVel), 0);
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

    public void resetOdometry(Pose2d pose) {
        super.resetOdometry(pose);

        m_FL.resetEncoder();
        m_BL.resetEncoder();
        m_FR.resetEncoder();
        m_BR.resetEncoder();
    }

    public static NEODrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new NEODrivetrain();
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
