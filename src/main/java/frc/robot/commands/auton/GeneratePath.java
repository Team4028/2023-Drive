// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utilities.drive.BeakDrivetrain;

public class GeneratePath extends CommandBase {
    private final Supplier<Pose2d> m_desiredPoseSupplier;
    private Pose2d m_desiredPose;
    private final BeakDrivetrain m_drivetrain;

    private SequentialCommandGroup m_trajectoryCommand;

    /** Generate and run a trajectory to the desired pose (field relative). */
    public GeneratePath(Supplier<Pose2d> desiredPose, BeakDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_desiredPoseSupplier = desiredPose;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = m_drivetrain.getPoseMeters();
        m_desiredPose = m_desiredPoseSupplier.get();

        if (robotPose.equals(m_desiredPose)) {
            this.cancel();
            return;
        }

        // Initialize path constraints (quarter-speed)
        PathConstraints constraints = new PathConstraints(m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25,
        m_drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25);

        // Add our path points--start at the current robot pose and end at the desired pose.
        List<PathPoint> points = new ArrayList<PathPoint>();

        points.add(new PathPoint(robotPose.getTranslation(), robotPose.getRotation(), robotPose.getRotation()));
        points.add(new PathPoint(m_desiredPose.getTranslation(), m_desiredPose.getRotation(), m_desiredPose.getRotation()));

        // Path planner magic
        PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, points);

        m_trajectoryCommand = m_drivetrain.getGeneratedTrajectoryCommand(traj);

        // debug
        Field2d field = new Field2d();

        field.setRobotPose(m_desiredPose);
        SmartDashboard.putData("BRUH FIELD", field);

        m_trajectoryCommand.schedule();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_trajectoryCommand.end(interrupted);

        SmartDashboard.putNumber("X Error", m_drivetrain.getPoseMeters().getX() - m_desiredPose.getX());
        SmartDashboard.putNumber("Y Error", m_drivetrain.getPoseMeters().getY() - m_desiredPose.getY());
        SmartDashboard.putNumber("Theta Error", m_drivetrain.getPoseMeters().getRotation().getDegrees() - m_desiredPose.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        if (m_trajectoryCommand == null) {
            return false;
        }
        return m_trajectoryCommand.isFinished();
    }
}
