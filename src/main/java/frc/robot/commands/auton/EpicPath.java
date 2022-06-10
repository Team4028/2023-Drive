// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.utilities.Util;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class EpicPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public EpicPath(BeakDifferentialDrivetrain drivetrain) {
        super.addCommands(
                Util.getTrajectoryCommand(Trajectories.Ball1(drivetrain), drivetrain),
                Util.getTrajectoryCommand(Trajectories.Ball2(drivetrain), drivetrain),
                new RotateDrivetrainToTargetPosition(324, 162, drivetrain));
        super.setInitialPose(Trajectories.Ball1(drivetrain));
    }
}
