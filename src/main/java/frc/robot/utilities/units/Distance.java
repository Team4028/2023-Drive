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
     * @param meters
     */
    public Distance(double meters) {
        m_distance = meters;
    }

    /**
     * Construct a distance from an inch value.
     * @param inches
     */
    public void fromInches(double inches) {
        m_distance = Units.inchesToMeters(inches);
    }

    /**
     * Construct a distance from a feet value.
     * @param feet
     */
    public void fromFeet(double feet) {
        m_distance = Units.feetToMeters(feet);
    }

    /**
     * Construct a distance from a centimeter value.
     * @param cm
     */
    public void fromCentimeters(double cm) {
        m_distance = cm / 100.;
    }

    /**
     * Get the distance in inches.
     * @return
     */
    public double getAsInches() {
        return Units.metersToInches(m_distance);
    }

    /**
     * Get the distance in feet.
     * @return
     */
    public double getAsFeet() {
        return Units.metersToFeet(m_distance);
    }

    /**
     * Get the distance in centimeters.
     * @return
     */
    public double getAsCentimeters() {
        return m_distance * 100.;
    }
}
