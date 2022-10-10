// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/** Get auton trajectories from paths. */
public class Trajectories {
    public static PathPlannerTrajectory TestPath(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("TestPath",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory Ball1(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Ball1",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }

    public static PathPlannerTrajectory Ball2(BeakDrivetrain drivetrain) {
        return PathPlanner.loadPath("Ball2",
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    }
    // public static Trajectory getTrajectory(String path) {
    // Trajectory traj = new Trajectory();
    // try {
    // Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("output/" +
    // path + ".wpilib.json");
    // traj = TrajectoryUtil.fromPathweaverJson(trajPath);
    // } catch (IOException e) {
    // System.out.println("Failed to load path " + path);
    // }

    // return traj;
    // }
}
