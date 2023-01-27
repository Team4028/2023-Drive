// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class JPath1 extends BeakAutonCommand {
    private Pose2d m_desiredAprTagPose;

    /** Creates a new TestPath. */
    public JPath1(Vision vision, BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.JPath1(drivetrain)),
                new InstantCommand(() -> m_desiredAprTagPose = vision.getTargetPose(drivetrain.getPoseMeters(),
                        new Transform3d(new Translation3d(Units.inchesToMeters(54.), Units.inchesToMeters(-0.), 0.),
                                new Rotation3d(0., 0., Units.degreesToRadians(0.))))),
                new WaitCommand(0.1),
                new GeneratePath(() -> m_desiredAprTagPose, drivetrain),
                new InstantCommand(() -> SmartDashboard.putNumber("pose", m_desiredAprTagPose.getRotation().getDegrees())),
                new InstantCommand(() -> System.out.println("DEEZ\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\ndeez")));
                // new RotateDrivetrainToAngle(() -> m_desiredAprTagPose.getRotation(), drivetrain, false));
        super.setInitialPose(Trajectories.JPath1(drivetrain));
    }
}
