// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class JPath1 extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public JPath1(Vision vision, BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.JPath1(drivetrain)),
                new WaitCommand(0.3),
                new GeneratePath(
                () -> vision.getTargetPose(drivetrain.getPoseMeters(),
                        new Transform3d(new Translation3d(Units.inchesToMeters(52.), Units.inchesToMeters(-0.), 0.),
                                new Rotation3d())),
                                drivetrain));
        super.setInitialPose(Trajectories.JPath1(drivetrain));
    }
}
