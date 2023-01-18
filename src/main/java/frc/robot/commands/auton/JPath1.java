// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class JPath1 extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public JPath1(BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.JPath1(drivetrain)),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(0.), drivetrain, isFinished()));
        super.setInitialPose(Trajectories.JPath1(drivetrain));
    }
}
