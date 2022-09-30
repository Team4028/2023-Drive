// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.units;

import edu.wpi.first.math.util.Units;

/** Class representing a distance in any unit. */
public class Distance {
    /** Internally represent as meters. */
    private double m_distance;

    /**
     * Create a distance from a meter value.
     */
    public Distance(double meters) {
        m_distance = meters;
    }

    /**
     * Construct a distance from an inch value.
     */
    public void fromInches(double inches) {
        m_distance = Units.inchesToMeters(inches);
    }

    /**
     * Construct a distance from a feet value.
     */
    public void fromFeet(double feet) {
        m_distance = Units.feetToMeters(feet);
    }

    /**
     * Construct a distance from a centimeter value.
     */
    public void fromCentimeters(double cm) {
        m_distance = cm / 100.;
    }

    /**
     * Get the distance in meters.
     */
    public double getAsMeters() {
        return m_distance;
    }

    /**
     * Get the distance in inches.
     */
    public double getAsInches() {
        return Units.metersToInches(m_distance);
    }

    /**
     * Get the distance in feet.
     */
    public double getAsFeet() {
        return Units.metersToFeet(m_distance);
    }

    /**
     * Get the distance in centimeters.
     */
    public double getAsCentimeters() {
        return m_distance * 100.;
    }
}
