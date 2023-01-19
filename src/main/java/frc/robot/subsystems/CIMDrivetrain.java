// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
import frc.robot.sim.CTREPhysicsSim;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.drive.RobotPhysics;
import frc.robot.utilities.motor.BeakTalonSRX;
import frc.robot.utilities.units.AngularVelocity;
import frc.robot.utilities.units.Distance;
import frc.robot.utilities.units.Velocity;

/** Add your docs here. */
public class CIMDrivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakTalonSRX m_FL, m_BL, m_FR, m_BR;

    private static final double kP = 0.01;
    private static final double kD = 0.02;

    private static final double AUTON_kP = 5.;
    private static final double[] AUTON_DRIVE_GAINS = { AUTON_kP, 0., 0. };

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int FR_ID = 3;
    private static final int BR_ID = 4;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(17.8);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(26.);
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(28.);

    private static final Distance WHEEL_DIAMETER = Distance.fromInches(6.258);
    private static final double GEAR_RATIO = 7.5;

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            1.1161,
            .17294,
            0.087223);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            new AngularVelocity(),
            TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER,
            GEAR_RATIO,
            FEED_FORWARD);

    private static CIMDrivetrain m_instance;

    public CIMDrivetrain() {
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

        m_FL = new BeakTalonSRX(FL_ID);
        m_BL = new BeakTalonSRX(BL_ID);
        m_FR = new BeakTalonSRX(FR_ID);
        m_BR = new BeakTalonSRX(BR_ID);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            double accel = 0.5;
            double maxVel = 44788.395;

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
                TRACK_WIDTH.getAsMeters(),
                null);

        m_FL.setDistancePerPulse(m_wheelDiameter, /* m_gearRatio */1);
        m_FR.setDistancePerPulse(m_wheelDiameter, /* m_gearRatio */1);
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        double maxVel = 44788.395;

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
        DifferentialDriveWheelSpeeds speeds = calcWheelSpeeds(x, rot);

        m_FL.setRate(speeds.leftMetersPerSecond, m_feedForward.calculate(speeds.leftMetersPerSecond), 0);
        m_FR.setRate(speeds.rightMetersPerSecond, m_feedForward.calculate(speeds.rightMetersPerSecond), 0);
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
