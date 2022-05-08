// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

// TODO: Set slots
/** Add your docs here. */
public class BeakTalon extends BaseTalon implements BeakMotorController {
    private int encoderCPR;

    public BeakTalon(int port, BeakTalonType type, String canBus) {
        super(port, type.value, canBus);

        switch (type) {
            case TalonFX: {
                super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                encoderCPR = 2048;
                break;
            }
            case TalonSRX: {
                super.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                encoderCPR = 4096;
                break;
            }
            default: {
                break;
            }
        }
    }

    public BeakTalon(int port, BeakTalonType type) {
        this(port, type, "");
    }

    public enum BeakTalonType {
        TalonFX("Talon FX"),
        TalonSRX("Talon SRX");

        public final String value;

        BeakTalonType(String initValue)
        {
            this.value = initValue;
        }
    }

    @Override
    public void set(double percentOutput) {
        super.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public double get() {
        return super.getMotorOutputPercent();
    }

    @Override
    public void setBrake(boolean brake) {
        super.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setVelocityRPM(double rpm) {
        setVelocityNU(rpm * encoderCPR / 600);
    }

    @Override
    public void setVelocityNU(double nu) {
        super.set(ControlMode.Velocity, nu);
    }

    @Override
    public void setPositionMotorRotations(double rotations) {
        setPositionNU(rotations * encoderCPR);
    }

    @Override
    public void setPositionNU(double nu) {
        super.set(ControlMode.Position, nu);
    }
    
    @Override
    public void setEncoderPositionMotorRotations(double rotations) {
        setEncoderPositionNU(rotations * encoderCPR);
    }

    @Override
    public void setEncoderPositionNU(double nu) {
        super.setSelectedSensorPosition(nu);
    }

    @Override
    public void setMotionMagicMotorRotations(double rotations) {
        setMotionMagicNU(rotations * encoderCPR);
    }

    @Override
    public void setMotionMagicNU(double nu) {
        super.set(ControlMode.MotionMagic, nu);
    }

    @Override
    public double getVelocityRPM() {
        return getVelocityNU() * encoderCPR / 600;
    }

    @Override
    public double getVelocityNU() {
        return super.getSelectedSensorVelocity();
    }

    @Override
    public double getPositionMotorRotations() {
        return getPositionNU() * encoderCPR;
    }

    @Override
    public double getPositionNU() {
        return super.getSelectedSensorPosition();
    }

    @Override
    public double getOutputVoltage() {
        return super.getMotorOutputVoltage();
    }

    public SlotConfiguration getPIDF(int slot) {
        SlotConfiguration config = new SlotConfiguration();
        super.getSlotConfigs(config, slot, 50);
        return config;
    }

    @Override
    public double getP(int slot) {
        return getPIDF(slot).kP;
    }

    @Override
    public double getI(int slot) {
        return getPIDF(slot).kI;
    }

    @Override
    public double getD(int slot) {
        return getPIDF(slot).kD;
    }

    @Override
    public double getF(int slot) {
        return getPIDF(slot).kF;
    }

    public TalonSRXSimCollection getTalonSRXSimCollection() {
        return super.getTalonSRXSimCollection();
    }

    public TalonFXSimCollection getTalonFXSimCollection() {
        return super.getTalonFXSimCollection();
    }

    @Override
    public void setP(double p, int slot) {
        super.config_kP(slot, p);
    }

    @Override
    public void setI(double i, int slot) {
        super.config_kI(slot, i);
    }

    @Override
    public void setD(double d, int slot) {
        super.config_kD(slot, d);
    }

    @Override
    public void setF(double f, int slot) {
        super.config_kF(slot, f);
    }

    @Override
    public double calculateFeedForward(double percentOutput, double desiredOutputNU) {
        double pidControllerOutput = percentOutput / 1023;
        return pidControllerOutput / desiredOutputNU;
    }

    @Override
    public double getVelocityEncoderCPR() {
        return encoderCPR;
    }

    @Override
    public double getPositionEncoderCPR() {
        return encoderCPR;
    }
}
