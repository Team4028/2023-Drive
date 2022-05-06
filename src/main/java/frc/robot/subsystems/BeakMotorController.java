// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;


/** Add your docs here. */
public interface BeakMotorController extends MotorController {
    public void setPIDF(
        double p,
        double i,
        double d,
        double f,
        int slot
    );

    public void set(double percentOutput);

    public double get();

    public void setBrake(boolean brake);

    public void setVelocityRPM(double rpm);

    public void setVelocityNU(double nu);

    public void setPositionMotorRotations(double rotations);

    public void setPositionNU(double nu);

    public void setMotionMagicMotorRotations(double rotations);

    public void setMotionMagicNU(double nu);

    public double getVelocityRPM();
    
    public double getVelocityNU();
    
    public double getPositionMotorRotations();
    
    public double getPositionNU();
    
    public void stop();

    public void setInverted(boolean inverted);

    public void setVoltage(double voltage);

    public double getBusVoltage();
    
    public double getOutputVoltage();

    public double getP(int slot);
    
    public double getI(int slot);

    public double getD(int slot);

    public double getF(int slot);    
}
