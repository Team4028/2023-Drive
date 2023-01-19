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
import frc.robot.utilities.units.AngularVelocity;
import frc.robot.utilities.units.Distance;
import frc.robot.utilities.units.Velocity;

/** Add your docs here. */
public class SixNEODrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakSparkMAX m_FL, m_BL, m_BL2, m_FR, m_BR, m_BR2;

    private static final double kP = 0.01;
    private static final double kD = 0.02;

    private static final double AUTON_kP = 0.1;
    private static final double[] AUTON_DRIVE_GAINS = { AUTON_kP, 0., 0. };

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int BL2_ID = 3;
    private static final int FR_ID = 4;
    private static final int BR_ID = 5;
    private static final int BR2_ID = 6;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(20.155);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(26.);
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(28.);

    private static final Distance WHEEL_DIAMETER = Distance.fromInches(6.258);
    private static final double GEAR_RATIO = 7.5;

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            0,
            0,
            0);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            new AngularVelocity(),
            TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER,
            GEAR_RATIO,
            FEED_FORWARD);

    private static SixNEODrivetrain m_instance;

    public SixNEODrivetrain() {
        super(
                PHYSICS,
                PIDConstants.Theta.gains,
                AUTON_DRIVE_GAINS,
                false);

        m_gyro = new AHRS(SPI.Port.kMXP);
        if (Robot.isSimulation()) {
            m_gyroSim = new AnalogGyroSim(0);
        }

        m_odom = new DifferentialDriveOdometry(getGyroRotation2d(), 0., 0.);

        m_FL = new BeakSparkMAX(FL_ID);
        m_BL = new BeakSparkMAX(BL_ID);
        m_BL2 = new BeakSparkMAX(BL2_ID);
        m_FR = new BeakSparkMAX(FR_ID);
        m_BR = new BeakSparkMAX(BR_ID);
        m_BR2 = new BeakSparkMAX(BR2_ID);

        m_BL.follow(m_FL);
        m_BL2.follow(m_FL);
        m_BR.follow(m_FR);
        m_BR2.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            // Theoretical value (given 3 neos)
            float stallTorque = 2.6f;
            float maxVel = 7380.63f;

            REVPhysicsSim.getInstance().addSparkMax(m_FL, stallTorque, maxVel);
            REVPhysicsSim.getInstance().addSparkMax(m_BL, stallTorque, maxVel);
            REVPhysicsSim.getInstance().addSparkMax(m_BL2, stallTorque, maxVel);
            REVPhysicsSim.getInstance().addSparkMax(m_FR, stallTorque, maxVel);
            REVPhysicsSim.getInstance().addSparkMax(m_BR, stallTorque, maxVel);
            REVPhysicsSim.getInstance().addSparkMax(m_BR2, stallTorque, maxVel);
        }

        sim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(3),
                7.5,
                0.9,
                Units.lbsToKilograms(60.),
                Units.inchesToMeters(3.),
                TRACK_WIDTH.getAsMeters(),
                null);

        m_FL.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
        m_BL.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
        m_BL2.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
        m_FR.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
        m_BR.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
        m_BR2.setDistancePerPulse(m_wheelDiameter, 1);// m_gearRatio);
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        double maxVel = 7380.63;

        m_FL.setPIDF(kP, 0., kD, m_FL.calculateFeedForward(1, maxVel), 0);
        m_BL.setPIDF(kP, 0., kD, m_BL.calculateFeedForward(1, maxVel), 0);
        m_BL2.setPIDF(kP, 0., kD, m_BL2.calculateFeedForward(1, maxVel), 0);
        m_FR.setPIDF(kP, 0., kD, m_FR.calculateFeedForward(1, maxVel), 0);
        m_BR.setPIDF(kP, 0., kD, m_BR.calculateFeedForward(1, maxVel), 0);
        m_BR2.setPIDF(kP, 0., kD, m_BR2.calculateFeedForward(1, maxVel), 0);
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
        DifferentialDriveWheelSpeeds speeds = calcWheelSpeeds(x, rot);

        m_FL.setRate(speeds.leftMetersPerSecond);
        m_BL.setRate(speeds.leftMetersPerSecond);
        m_BL2.setRate(speeds.leftMetersPerSecond);
        m_FR.setRate(speeds.rightMetersPerSecond);
        m_BR.setRate(speeds.rightMetersPerSecond);
        m_BR2.setRate(speeds.rightMetersPerSecond);

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

        field.setRobotPose(sim.getPose());
        SmartDashboard.putData(field);
    }
}
