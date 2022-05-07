// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class BeakTalonSRX extends BeakTalon {
    public BeakTalonSRX(int port, String canBus) {
        super(port, BeakTalonType.TalonSRX, canBus);
    }

    public BeakTalonSRX(int port) {
        super(port, BeakTalonType.TalonSRX, "");
    }
}
