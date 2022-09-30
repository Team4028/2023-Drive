// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.units;

import edu.wpi.first.math.util.Units;

/** Class representing a velocity in any unit. */
public class Velocity {
    /** Internally represent as meters per second. */
    private double m_velocity;

    /**
     * Create a velocity from a meters per second value.
     */
    public Velocity(double meters) {
        m_velocity = meters;
    }

    /**
     * Construct a velocity from an inches per second value.
     */
    public void fromInchesPerSecond(double inches) {
        m_velocity = Units.inchesToMeters(inches);
    }

    /**
     * Construct a velocity from a feet per second value.
     */
    public void fromFeetPerSecond(double feet) {
        m_velocity = Units.feetToMeters(feet);
    }

    /**
     * Construct a velocity from a centimeters per second value.
     */
    public void fromCentimetersPerSecond(double cm) {
        m_velocity = cm / 100.;
    }

    /**
     * Construct a velocity from a miles per hour value.
     */
    public void fromMilesPerHour(double mph) {
        /* Converts to feet per second first. */
        m_velocity = Units.feetToMeters(mph / 5280. / 3600.);
    }

    /**
     * Get the velocity in meters per second.
     */
    public double getAsMetersPerSecond() {
        return m_velocity;
    }

    /**
     * Get the velocity in inches per second.
     */
    public double getAsInchesPerSecond() {
        return Units.metersToInches(m_velocity);
    }

    /**
     * Get the velocity in feet per second.
     */
    public double getAsFeetPerSecond() {
        return Units.metersToFeet(m_velocity);
    }

    /**
     * Get the velocity in centimeters per second.
     */
    public double getAsCentimetersPerSecond() {
        return m_velocity * 100.;
    }

    /**
     * Get the velocity in miles per hour.
     */
    public double getAsMilesPerHour() {
        /* Convert to feet per hour first. */
        return Units.metersToFeet(m_velocity * 3600.) * 5280.;
    }
}
