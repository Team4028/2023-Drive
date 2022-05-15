// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Comprehensive documentation for all methods
public class BeakLimelight extends SubsystemBase {
    public static final double PIPELINE = 0;

    /* Height of the goal, in inches--based on the center of the reflective tape */
    public static final double TARGET_HEIGHT = 104.; // Rapid React
    
    /* Mounting height of the Limelight, in inches--center of the lens to the floor */
    public static final double MOUNT_HEIGHT = 0.;

    public static final double HEIGHT_DELTA = TARGET_HEIGHT - MOUNT_HEIGHT;

    /* Mounting angle of the Limelight, in degrees, from perfect-vertical--see Limelight's docs
     * for a way to calculate a theoretical angle. You can also make guesses, and in either case,
     * ensure to tune further. */
    public static final double MOUNT_ANGLE = 0.;

    protected NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry tx = entry("tx");
    protected NetworkTableEntry ta = entry("ta");
    protected NetworkTableEntry tv = entry("tv");
    protected NetworkTableEntry tshort = entry("tshort");
    protected NetworkTableEntry tlong = entry("tlong");
    protected NetworkTableEntry thor = entry("thor");
    protected NetworkTableEntry tvert = entry("tvert");
    protected NetworkTableEntry ty = entry("ty");
    protected NetworkTableEntry ts = entry("ts");
    protected NetworkTableEntry stream = entry("stream");
    protected NetworkTableEntry ledMode = entry("ledMode");
    protected NetworkTableEntry pipeline = entry("pipeline");

    /** Creates a new Limelight. */
    public BeakLimelight() {
        setPipeline(0);
        setPictureInPicture(0);
        setLedMode(1);
    }

    public double getX() {
        return tx.getDouble(0);
    }

    public double getXForRot() {
        return tx.getDouble(0.0);
    }

    /**
     * Get the Limelight's LED mode.
     * 
     * @return LED mode.</p>
     *
     *         0 = use pipeline's LED mode</p>
     * 
     *         1 = force off</p>
     * 
     *         2 = blink</p>
     * 
     *         3 = force on</p>
     */
    public double getLedMode() {
        return ledMode.getDouble(0);
    }

    /**
     * Set the Limelight's LED mode.
     * 
     * @param mode LED mode.</p>
     *
     *             0 = use pipeline's LED mode</p>
     * 
     *             1 = force off</p>
     * 
     *             2 = blink</p>
     * 
     *             3 = force on</p>
     */
    public void setLedMode(double mode) {
        ledMode.forceSetDouble(mode);
    }

    public void toggleLedMode() {
        setLedMode(Math.abs(ledMode.getDouble(0.0) - 1.0));
    }

    public double getArea() {
        return ta.getDouble(0);
    }

    public double getBoxShortLength() {
        return tshort.getDouble(0);
    }

    public double getBoxLongLength() {
        return tlong.getDouble(0);
    }

    public double getHorBoxLength() {
        return thor.getDouble(0);
    }

    public double getVertBoxLength() {
        return tvert.getDouble(0);
    }

    public double getSkew() {
        return ts.getDouble(0);
    }

    public double getY() {
        return ty.getDouble(0);
    }

    public void setPipeline(double pipe) {
        pipeline.setDouble(pipe);
    }

    public void setPictureInPicture(double mode) {
        stream.setDouble(mode);
    }

    public boolean getHasTarget() {
        return tv.getDouble(0.0) != 0.0;
    }

    /**
     * Get the distance calculated from the Limelight.
     * @return Distance, in feet.
     */
    public double getDistance() {
        double goalAngle = (MOUNT_ANGLE + getY()) * (Math.PI / 180.);
        double dist = HEIGHT_DELTA / (Math.tan(goalAngle));

        return dist / 12.0;
    }

    public double getRoundedXOffset() {
        return Math.round(getX() * 100.) / 100.;
    }

    public boolean set(String key, double val) {
        return entry(key).setNumber(val);
    }

    public NetworkTableEntry entry(String key) {
        return nt.getEntry(key);
    }

    public double get(String key, double defaultValue) {
        return SmartDashboard.getNumber(key, defaultValue);
    }
}