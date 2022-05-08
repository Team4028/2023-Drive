// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/** Add your docs here. */
public class BeakSparkMAX extends CANSparkMax implements BeakMotorController {
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;

    public BeakSparkMAX(int port) {
        super(port, MotorType.kBrushless);
        
        resetControllers();
    }

    private void resetControllers() {
        encoder = super.getEncoder();
        pid = super.getPIDController();
    }

    @Override
    public void setBrake(boolean brake) {
        super.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setVelocityRPM(double rpm) {
        setVelocityNU(rpm);
    }

    @Override
    public void setVelocityNU(double nu) {
        pid.setReference(nu, ControlType.kVelocity);
    }

    @Override
    public void setPositionMotorRotations(double rotations) {
        setPositionNU(rotations * encoder.getCountsPerRevolution());
    }

    @Override
    public void setPositionNU(double nu) {
        pid.setReference(nu, ControlType.kPosition);
    }

    @Override
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * encoder.getCountsPerRevolution());
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        encoder.setPosition(nu);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations) {
        setMotionMagicNU(rotations);
    }

    @Override
    public void setMotionMagicNU(double nu) {
        throw new RuntimeException("REV Spark MAX does not support Motion Magic.");
    }

    @Override
    public double getVelocityRPM() {
        return getVelocityNU();
    }

    @Override
    public double getVelocityNU() {
        return encoder.getVelocity();
    }

    @Override
    public double getPositionMotorRotations() {
        return getPositionNU() / encoder.getCountsPerRevolution();
    }

    @Override
    public double getPositionNU() {
        return encoder.getPosition();
    }

    @Override
    public double getP(int slot) {
        return pid.getP(slot);
    }

    @Override
    public double getI(int slot) {
        return pid.getI(slot);
    }

    @Override
    public double getD(int slot) {
        return pid.getD(slot);
    }

    @Override
    public double getF(int slot) {
        return pid.getFF(slot);
    }

    @Override
    public void setP(double p, int slot) {
        pid.setP(p, slot);
    }

    @Override
    public void setI(double i, int slot) {
        pid.setI(i, slot);        
    }

    @Override
    public void setD(double d, int slot) {
        pid.setD(d, slot);
    }

    @Override
    public void setF(double f, int slot) {
        pid.setFF(f, slot);
    }

    @Override
    public double calculateFeedForward(double percentOutput, double desiredOutputNU) {
        return percentOutput / desiredOutputNU;
    }

    @Override
    public double getVelocityEncoderCPR() {
        //return encoder.getCountsPerRevolution();
        return 600.; // TEMP
    }

    @Override
    public double getPositionEncoderCPR() {
        // return encoder.getCountsPerRevolution();
        return 1.; // TEMP
    }

    @Override
    public double getOutputVoltage() {
        return super.getAppliedOutput() * super.getBusVoltage();
    }

    @Override
    public void follow(int leaderCANId) {
        super.follow(ExternalFollower.kFollowerSparkMax, leaderCANId);
    }
}
