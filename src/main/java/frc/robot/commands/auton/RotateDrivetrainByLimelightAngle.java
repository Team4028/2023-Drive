// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            4.4,
            0.011,
            0.4, // This is the best tuning solution I've found but it still needs a lot of work.
            // This currently gets to about 1 degree off but it needs to be like 0.1.
            // Behavior is relatively consistent; seems to be 1 degree overshooting.
            // Also it sometimes oscillates. God save me
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond() * 0.1,
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond() * 0.25)), // get this stuff from dreivetrain
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
    getController().setTolerance(Math.toRadians(0.75));

    SmartDashboard.putNumber("P gain", getController().getP());
    SmartDashboard.putNumber("I gain", getController().getI());
    SmartDashboard.putNumber("D gain", getController().getD());
    SmartDashboard.putNumber("Goal", getController().getGoal().position);
  }

  @Override
  public void execute() {
    super.execute();

    double p = SmartDashboard.getNumber("P gain", getController().getP());
    double i = SmartDashboard.getNumber("I gain", getController().getP());
    double d = SmartDashboard.getNumber("D gain", getController().getP());
    double goal = SmartDashboard.getNumber("Goal", getController().getGoal().position);

    if (p != getController().getP()) {
      getController().setP(p);
    }

    if (i != getController().getI()) {
      getController().setI(i);
    }

    if (d != getController().getD()) {
      getController().setD(d);
    }

    if (goal != getController().getGoal().position) {
      getController().setGoal(goal);
    }

    SmartDashboard.putNumber("Setpoint", getController().getSetpoint().velocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
