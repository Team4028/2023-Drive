// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.drive.BeakDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainByLimelightAngle extends ProfiledPIDCommand {
  /** Creates a new RotateDrivetrainByLimelightAngle. */
  public RotateDrivetrainByLimelightAngle(Limelight limelight, BeakDrivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            drivetrain.getThetaController().getP() * 0.8,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond(),
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond())), // get this stuff from dreivetrain
        // This should return the measurement
        () -> Math.toRadians(limelight.getX()),
        // This should return the goal (can also be a constant)
        0.0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(
            0,
            0,
            (output + setpoint.velocity) / drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond()); // TODO: driveRaw()
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain, limelight);
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Math.toRadians(1.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
