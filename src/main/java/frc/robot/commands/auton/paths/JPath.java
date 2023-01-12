// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.paths;

import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.auton.MoveDrivetrainToTargetDistance;
import frc.robot.commands.auton.RotateDrivetrainByLimelightAngle;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.drive.Trajectories;
import frc.robot.utilities.units.Distance;

public class JPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public JPath(Limelight limelight, BeakDrivetrain drivetrain) {
        super.addCommands(
                drivetrain.getTrajectoryCommand(Trajectories.JStop1(drivetrain)),
                drivetrain.getTrajectoryCommand(Trajectories.JStop2(drivetrain)),
                new RotateDrivetrainByLimelightAngle(limelight, drivetrain).withTimeout(1.0),
                new MoveDrivetrainToTargetDistance(Distance.fromFeet(2.0), limelight, drivetrain));
        super.setInitialPose(Trajectories.JStop1(drivetrain));
    }
}
