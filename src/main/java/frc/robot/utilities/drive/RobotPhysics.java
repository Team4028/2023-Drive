// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class RobotPhysics {
    public double maxVelocity;
    public double maxAngularVelocity;

    public double trackWidth;
    public double wheelBase;

    public double wheelDiameter;

    public double driveGearRatio;

    public SimpleMotorFeedforward feedforward;

    /**
     * Construct a new physics object to pass to a drivetrain.
     * 
     * @param maxVelocity        Maximum travel velocity of the robot, in meters per
     *                           second.
     * @param maxAngularVelocity Maximum angular velocity of the robot, in radians
     *                           per second. Set to 0 to calculate a theoretical.
     * @param trackWidth         Distance from the left wheels to the right wheels,
     *                           in inches.
     * @param wheelBase          Distance from the front wheels to the back wheels,
     *                           in inches.
     * @param wheelDiameter      Diameter of the wheels, in inches.
     * @param driveGearRatio     Gear ratio of the drive motors.
     * @param feedforward        A {@link SimpleMotorFeedforward} calculated from
     *                           SysId.
     */
    public RobotPhysics(double maxVelocity, double maxAngularVelocity, double trackWidth, double wheelBase,
            double wheelDiameter, double driveGearRatio, SimpleMotorFeedforward feedforward) {
        this.maxVelocity = maxVelocity;
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.maxAngularVelocity = (maxAngularVelocity == 0. ? calcTheoreticalAngularVelocity() : maxAngularVelocity);
        this.wheelDiameter = wheelDiameter;
        this.driveGearRatio = driveGearRatio;
        this.feedforward = feedforward;
    }

    protected double calcTheoreticalAngularVelocity() {
        return maxVelocity / Math.hypot(Units.inchesToMeters(trackWidth) / 2.0, Units.inchesToMeters(wheelBase) / 2.0);
    }
}
