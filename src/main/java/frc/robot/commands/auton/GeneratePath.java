// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utilities.drive.BeakDrivetrain;

public class GeneratePath extends CommandBase {
    private final Pose2d m_desiredPose;
    private final BeakDrivetrain m_drivetrain;

    private SequentialCommandGroup m_trajectoryCommand;

    /** Generate and run a trajectory to the desired pose (field relative). */
    public GeneratePath(Pose2d desiredPose, BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_desiredPose = desiredPose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        PathConstraints constraints = new PathConstraints(m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
            m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());

        Pose2d robotPose = m_drivetrain.getPoseMeters();

        List<PathPoint> points = new ArrayList<PathPoint>();

        points.add(new PathPoint(robotPose.getTranslation(), robotPose.getRotation(), robotPose.getRotation()));
        points.add(new PathPoint(m_desiredPose.getTranslation(), m_desiredPose.getRotation(), m_desiredPose.getRotation()));

        PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, points);

        m_trajectoryCommand = m_drivetrain.getTrajectoryCommand(traj);

        m_trajectoryCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return m_trajectoryCommand.isFinished();
    }
}
