// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.wpilibj.RobotController;

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
    public void setPIDF(double p, double i, double d, double f, int slot) {
        super.config_kP(slot, p);
        super.config_kI(slot, i);
        super.config_kD(slot, d);
        super.config_kF(slot, f);
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
    public void stop() {
        set(0.);
    }

    @Override
    public void setVoltage(double voltage) {
        set(voltage / RobotController.getBatteryVoltage());
    }

    @Override
    public double getBusVoltage() {
        return super.getBusVoltage();
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
}
