// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.utilities.BeakSparkMAX;
import frc.robot.utilities.Util;

public class NEODrivetrain extends SubsystemBase {
    private BeakSparkMAX m_FL, m_BL, m_FR, m_BR;

    private Gyro m_gyro;
    private DifferentialDriveOdometry m_odom;

    private Pose2d m_pose;
    Field2d field = new Field2d();

    private AnalogGyroSim m_gyroSim;

    private static NEODrivetrain m_instance;

    DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide,
        KitbotGearing.k5p95,
        KitbotWheelSize.kSixInch,
        null);/*new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),
        DriveConstants.GEAR_RATIO,
        7.5,
        Units.lbsToKilograms(125.),
        DriveConstants.WHEEL_DIAMETER / 2.,
        DriveConstants.TRACK_WIDTH,
        null
    );*/

    /* Config and initialization */
    public NEODrivetrain() {
        if (Robot.isSimulation()) {
            m_gyroSim = new AnalogGyroSim(1);
        } else {
            m_gyro = new AHRS(SPI.Port.kMXP);
        }
        m_odom = new DifferentialDriveOdometry(getGyroRotation());

        m_FL = new BeakSparkMAX(SubsystemConstants.DRIVE_FL);
        m_BL = new BeakSparkMAX(SubsystemConstants.DRIVE_BL);
        m_FR = new BeakSparkMAX(SubsystemConstants.DRIVE_FR);
        m_BR = new BeakSparkMAX(SubsystemConstants.DRIVE_BR);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_FL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BL, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_FR, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(m_BR, DCMotor.getNEO(1));
        }
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        // TODO: get these from SysId
        // for (BeakSparkMAX spark : motors) {
            // spark.setP(PIDConstants.Drive.kP, 0);
            // spark.setD(PIDConstants.Drive.kD, 0);
            // spark.setF(1 / 5676, 0);
        // }
        m_FL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FL.calculateFeedForward(1, 5676), 0);
        m_BL.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BL.calculateFeedForward(1, 5676), 0);
        m_FR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_FR.calculateFeedForward(1, 5676), 0);
        m_BR.setPIDF(PIDConstants.Drive.kP, 0., PIDConstants.Drive.kD, m_BR.calculateFeedForward(1, 5676), 0);
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

    /* Drive & Auton commands */

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x   Speed of the robot in the x direction (forward).
     * @param y   Speed of the robot in the y direction (sideways) -- UNUSED FOR NOW
     *            NOT SWERVE.
     * @param rot Angular rate of the robot.
     */
    public void drive(double x, double y, double rot) {
        x *= DriveConstants.MAX_VELOCITY;
        rot *= AutonConstants.MAX_ANGULAR_VELOCITY;
        DifferentialDriveWheelSpeeds speed = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(
                new ChassisSpeeds(x, 0, rot));

        double rightVel = speed.rightMetersPerSecond / DriveConstants.NEO_ENCODER_DISTANCE_PER_PULSE;
        double leftVel = speed.leftMetersPerSecond / DriveConstants.NEO_ENCODER_DISTANCE_PER_PULSE;

        speed.desaturate(DriveConstants.MAX_VELOCITY);

        if (x != 0. || rot != 0.) {
            if (Robot.isSimulation()) {
                // m_FL.set(speed.leftMetersPerSecond);
                // m_FR.set(speed.rightMetersPerSecond);
                // m_FL.set(ControlMode.Velocity, leftVel);
                m_FL.setVelocityRPM(leftVel);
                // DemandType.ArbitraryFeedForward,
                // DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) /
                // DriveConstants.NOMINAL_VOLTAGE);
                // m_FR.set(ControlMode.Velocity, rightVel);
                m_FR.setVelocityRPM(rightVel);
                // DemandType.ArbitraryFeedForward,
                // DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) /
                // DriveConstants.NOMINAL_VOLTAGE);
            } else {
                // m_FL.set(ControlMode.Velocity, leftVel,
                //         DemandType.ArbitraryFeedForward,
                //         DriveConstants.FEED_FORWARD.calculate(speed.leftMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
                // m_FR.set(ControlMode.Velocity, rightVel,
                //         DemandType.ArbitraryFeedForward,
                //         DriveConstants.FEED_FORWARD.calculate(speed.rightMetersPerSecond) / DriveConstants.NOMINAL_VOLTAGE);
            }
        } else {
            m_FL.setVelocityNU(0.);
            m_FR.setVelocityNU(0.);
        }
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            Util.NUtoMeters(m_FL.getVelocityNU(), DriveConstants.NEO_ENCODER_CPR, DriveConstants.GEAR_RATIO, Units.metersToInches(DriveConstants.WHEEL_DIAMETER)),
            Util.NUtoMeters(m_FR.getVelocityNU(), DriveConstants.NEO_ENCODER_CPR, DriveConstants.GEAR_RATIO, Units.metersToInches(DriveConstants.WHEEL_DIAMETER))
        );
                // DriveConstants.ENCODER_DISTANCE_PER_PULSE / m_FL.getSelectedSensorVelocity(),
                // DriveConstants.ENCODER_DISTANCE_PER_PULSE / m_FR.getSelectedSensorVelocity());
    }

    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_FR.setVoltage(right);
        // m_gyroSim.setAngle(m_gyroSim.getAngle() + DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond);
    }

    /**
     * Class for managing and driving a 4-motor
     * differential drivetrain, with 4 TalonSRX
     * controllers and a NavX.
     **/
    public static NEODrivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new NEODrivetrain();
        }
        return m_instance;
    }

    /* NavX wrapper methods */
    public Rotation2d getGyroRotation() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        } else {
            return m_gyro.getRotation2d();
        }
    }

    public void zero() {
        m_gyro.reset();
    }

    public double getGyroHeading() {
        return getGyroRotation().getDegrees();
    }

    public double getRate() {
        if (Robot.isSimulation()) {
            return m_gyroSim.getRate();
        } else {
            return m_gyro.getRate();
        }
    }

    /* Pose & Odometry */
    public Pose2d getPoseMeters() {
        return m_odom.getPoseMeters();
    }

    public Rotation2d getRotation() {
        return getPoseMeters().getRotation();
    }

    /**
     * Reset odometry to specified pose.
     * 
     * @param pose Pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odom.resetPosition(pose, getGyroRotation());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        sim.setInputs(
            m_FR.getAppliedOutput() * m_FR.getBusVoltage(),
            m_FL.getAppliedOutput() * m_FR.getBusVoltage());
        sim.update(0.02);

        m_gyroSim.setAngle(-sim.getHeading().getDegrees());
        Rotation2d rot = getGyroRotation();

        m_pose = m_odom.update(rot,
                Util.NUtoMeters(m_FL.getPositionNU(), 1, DriveConstants.GEAR_RATIO, Units.metersToInches(DriveConstants.WHEEL_DIAMETER)),
                Util.NUtoMeters(m_FR.getPositionNU(), 1, DriveConstants.GEAR_RATIO, Units.metersToInches(DriveConstants.WHEEL_DIAMETER))
        );

        field.setRobotPose(m_pose);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("Heading", getRotation().getDegrees());
        SmartDashboard.putNumber("X (meters)", m_pose.getX());
        SmartDashboard.putNumber("Y (meters)", m_pose.getY());
    }
}
