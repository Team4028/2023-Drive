// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Trajectories;
import frc.robot.Util;
import frc.robot.subsystems.BeakDifferentialDrivetrain;
import frc.robot.subsystems.TalonDrivetrain;

public class TestPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public TestPath(BeakDifferentialDrivetrain drivetrain) {
        super.addCommands(
                Util.getTrajectoryCommand(Trajectories.TestPath(), drivetrain),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), drivetrain, false));
        super.setInitialPose(Trajectories.TestPath());
    }
}
