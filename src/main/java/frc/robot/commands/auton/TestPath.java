// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Trajectories;
import frc.robot.Util;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public TestPath(Drivetrain drivetrain) {
        super.addCommands(
                Util.getTrajectoryCommand(Trajectories.TestPath(), drivetrain),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), drivetrain));
        super.setInitialPose(Trajectories.TestPath());
    }
}
