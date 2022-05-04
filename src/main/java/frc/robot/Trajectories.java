// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.TalonDriveConstants;

/** Get auton trajectories from paths. */
public class Trajectories {
    public static PathPlannerTrajectory TestPath() {
        return PathPlanner.loadPath("TestPath",
            TalonDriveConstants.MAX_VELOCITY,
            TalonDriveConstants.MAX_VELOCITY
        );
    }

    public static PathPlannerTrajectory Ball1() {
        return PathPlanner.loadPath("Ball1",
            TalonDriveConstants.MAX_VELOCITY,
            TalonDriveConstants.MAX_VELOCITY
        );
    }

    public static PathPlannerTrajectory Ball2() {
        return PathPlanner.loadPath("Ball2",
            TalonDriveConstants.MAX_VELOCITY,
            TalonDriveConstants.MAX_VELOCITY
        );
    }
    // public static Trajectory getTrajectory(String path) {
    //     Trajectory traj = new Trajectory();
    //     try {
    //         Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + path + ".wpilib.json");
    //         traj = TrajectoryUtil.fromPathweaverJson(trajPath);
    //     } catch (IOException e) {
    //         System.out.println("Failed to load path " + path);
    //     }

    //     return traj;
    // }
}
