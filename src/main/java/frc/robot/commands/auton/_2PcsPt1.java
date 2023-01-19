// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class JPath2 extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public JPath2(BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.JPath2(drivetrain)),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180), drivetrain, false));
        super.setInitialPose(Trajectories.JPath2(drivetrain));
    }
}
