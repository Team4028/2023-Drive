// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Common interface for all motor controllers. */
public interface BeakMotorController extends MotorController {
    /**
     * Set PIDF gains.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param f Feed-Forward gain.
     * @param slot The slot to set these values in.
     */
    default void setPIDF(
        double p,
        double i,
        double d,
        double f,
        int slot
    ) {
        setP(p, slot);
        setI(i, slot);
        setD(d, slot);
        setF(f, slot);
    }

    /** 
     * Set the motor to be on brake or coast mode.
     * @param brake True = brake, False = coast
     */
    public void setBrake(boolean brake);

    /**
     * Run the motor in velocity mode, in RPM.
     * @param rpm RPM to run.
     */
    public void setVelocityRPM(double rpm);

    /**
     * Run the motor in velocity mode, in NU.</p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * @param nu NU to run.
     */
    public void setVelocityNU(double nu);

    /**
     * Run the motor in position mode, in motor rotations.
     * @param rotations Rotations to run.
     */
    public void setPositionMotorRotations(double rotations);

    /**
     * Run the motor in position mode, in NU.</p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for SparkMAX.
     * @param nu NU to run.
     */
    public void setPositionNU(double nu);

    /**
     * Sets the encoder's position, in motor rotations.
     * @param rotations Rotations to set the encoder to.
     */
    public void setEncoderPositionMotorRotations(double rotations);

    /**
     * Sets the encoder's position, in NU.</p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for SparkMAX.
     * @param nu NU to set the encoder to.
     */
    public void setEncoderPositionNU(double nu);

    /**
     * Resets the encoder position to 0.
     */
    default void resetEncoder() {
        setEncoderPositionNU(0.);
    }

    /**
     * Runs the motor in motion magic mode, in motor rotations.</p>
     * Not currently supported by SparkMAX.
     * @param rotations Rotations to run.
     */
    public void setMotionMagicMotorRotations(double rotations);

    /**
     * Runs the motor in motion magic mode, in NU.</p>
     * Not currently supported by SparkMAX.</p>
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for SparkMAX.
     * @param nu NU to run.
     */
    public void setMotionMagicNU(double nu);

    /**
     * Get the motor velocity, in RPM.
     * @return Velocity in RPM.
     */
    public double getVelocityRPM();
    
    /**
     * Get the motor velocity, in NU.</p>
     * NU/100ms for Talons, RPM for SparkMAX.
     * @return Velocity in NU.
     */
    public double getVelocityNU();
    
    /**
     * Get the motor position, in motor rotations.
     * @return Position in motor rotations.
     */
    public double getPositionMotorRotations();
    
    /**
     * Get the motor position, in NU.
     * 2048 NU per rotation for TalonFX, 4096 for TalonSRX, and usually 1 for SparkMAX.
     * @return Position in NU.
     */
    public double getPositionNU();
    
    /**
     * Stop the motor.
     */
    default void stop() {
        set(0.);
    }

    @Override
    default void disable() {
        stop();
    }

    @Override
    default void stopMotor() {
        stop();
    }

    /**
     * Get the voltage currently being run to the motor controller.
     */
    public double getBusVoltage();
    
    /**
     * Get the current applied voltage to the motor controller.
     * @return Applied voltage.
     */
    default double getOutputVoltage() {
        return get() * getBusVoltage();
    }

    /**
     * Get the P value of the PID controller.
     * @param slot Slot to get from.
     * @return Proportional gain.
     */
    public double getP(int slot);
    
    /**
     * Get the I value of the PID controller.
     * @param slot Slot to get from.
     * @return Integral gain.
     */
    public double getI(int slot);

    /**
     * Get the D value of the PID controller.
     * @param slot Slot to get from.
     * @return Derivative gain.
     */
    public double getD(int slot);

    /**
     * Get the F value of the PID controller.
     * @param slot Slot to get from.
     * @return Feed-Forward gain.
     */
    public double getF(int slot); 
    
    /**
     * Set the P value of the PID controller.
     * @param p Proportional gain.
     * @param slot Slot to set to.
     */
    public void setP(double p, int slot);
    
    /**
     * Set the I value of the PID controller.
     * @param i Integral gain.
     * @param slot Slot to set to.
     */
    public void setI(double i, int slot);

    /**
     * Set the D value of the PID controller.
     * @param D Derivative gain.
     * @param slot Slot to set to.
     */
    public void setD(double d, int slot);

    /**
     * Set the F value of the PID controller.
     * @param f Feed-Forward gain.
     * @param slot Slot to set to.
     */
    public void setF(double f, int slot);

    /**
     * Calculate the desired feed-forward, given a percent output and the NU that the PID controller should return.</p>
     * Example process: Run the motor at 100% output (safely).</p>
     * Get the motor's velocity in NU (i.e. 22000 NU/100ms for a free-spinning Falcon, 11000 RPM for a free-spinning NEO 550)</p>
     * Pass 1 to percentOutput, and your recorded velocity to desiredOutputNU.
     * 
     * @param percentOutput Percent output of the motor (0-1).
     * @param desiredOutputNU Velocity in NU.
     * 
     * @return A calculated feed forward.</p>
     * For Talons, this will be in the 0.005-0.2 range.</p>
     * For SparkMAXes, this will be a very small number.
     */
    public double calculateFeedForward(double percentOutput, double desiredOutputNU);
}
