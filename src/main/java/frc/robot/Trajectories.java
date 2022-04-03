// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class Trajectories {
    public static Trajectory getTrajectory(String path) {
        Trajectory traj = new Trajectory();
        try {
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + path + ".wpilib.json");
            traj = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException e) {
            System.out.println("Failed to load path " + path);
        }

        return traj;
    } 

    public static Trajectory Bruh() {
        return getTrajectory("Bruh");
    }
}
