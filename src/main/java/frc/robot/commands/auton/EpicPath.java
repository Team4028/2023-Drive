// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Trajectories;
import frc.robot.utilities.Util;

public class EpicPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public EpicPath(Drivetrain drivetrain) {
        super.addCommands(
                Util.getTrajectoryCommand(Trajectories.Ball1(), drivetrain),
                Util.getTrajectoryCommand(Trajectories.Ball2(), drivetrain),
                new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.Ball2().getEndState().poseMeters)),
                new RotateDrivetrainToAngle(Rotation2d.fromDegrees(12.5), drivetrain, false));
        super.setInitialPose(Trajectories.Ball1());
    }
}
