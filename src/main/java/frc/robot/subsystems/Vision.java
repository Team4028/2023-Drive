// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.units.Distance;

public class Vision extends SubsystemBase {
    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;

    private static final String CAMERA_NAME = "Global_Shutter_Camera";

    private static Vision m_instance;

    /** Creates a new Vision. */
    public Vision() {
        m_camera = new PhotonCamera(CAMERA_NAME);

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }
    }

    /**
     * Get the best target from the camera.
     * @return The target data, or null if none are found.
     */
    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = m_camera.getLatestResult();
        
        boolean hasTarget = result.hasTargets();

        PhotonTrackedTarget target = null;

        if (hasTarget) {
            target = result.getBestTarget();
        }

        return target;
    }

    public Pose3d getLatestEstimatedRobotPose() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose;
            }
        }
        return new Pose3d();
    }

    /**
     * Gets the pose of a target.
     * @param robotPose The current robot pose.
     * @param distance The desired distance away from the tag (X direction only--forwards and backwards). Positive is backwards.
     * @return The pose of the specified distance from the target.
     */
    public Pose2d getTargetPose(Pose2d robotPose, Distance distance) {
        PhotonTrackedTarget target = getBestTarget();
        
        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Transform3d targetOffset = cameraToTarget.plus(new Transform3d(new Pose3d(), new Pose3d(-distance.getAsMeters(), 0, 0, new Rotation3d())));

            Pose3d pose = new Pose3d(robotPose);

            Pose3d scoringPose = pose.plus(targetOffset);

            return scoringPose.toPose2d();
        }

        return robotPose;
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
