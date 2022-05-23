// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A subsystem containing a gyro, with simulation support. */
public class BeakGyroSubsystem extends SubsystemBase {
    protected Gyro m_gyro;
    protected AnalogGyroSim m_gyroSim;

    /**
     * Gets the gyro's reported angle.
     * 
     * @return A {@link Rotation2d} containing the reported angle of the gyro.
     */
    public Rotation2d getGyroRotation2d() {
        if (TimedRobot.isSimulation()) {
            return Rotation2d.fromDegrees(m_gyroSim.getAngle());
        } else {
            return m_gyro.getRotation2d();
        }
    }

    /**
     * Zero the gyro.
     */
    public void resetGyro() {
        if (TimedRobot.isReal()) {
            m_gyro.reset();
        }
    }

    /**
     * Get the gyro's reported heading.
     * 
     * @return The reported angle of the gyro in degrees.
     */
    public double getGyroHeading() {
        return getGyroRotation2d().getDegrees();
    }

    /**
     * Get the gyro's reported rate.
     * 
     * @return The reported rate of the gyro in degrees per second.
     */
    public double getGyroRate() {
        if (TimedRobot.isSimulation()) {
            return m_gyroSim.getRate();
        } else {
            return m_gyro.getRate();
        }
    }
}
