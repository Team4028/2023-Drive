// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.units.Distance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveDrivetrainToTargetDistance extends ProfiledPIDCommand {
  private final static DoubleSupplier m_supp = () -> 19.;
  /** Creates a new MoveDrivetrainToTargetDistance. */
  public MoveDrivetrainToTargetDistance(double goalTargetDistance, Limelight limelight, BeakDrivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.4,
            0,
            0.06,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.5)),
        // This should return the measurement
        () -> limelight.getTargetDistance(),
        // This should return the goal (can also be a constant)
        m_supp,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(
              -(output + setpoint.velocity) / drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
              0.,
              0.,
              false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(limelight, drivetrain);
    // getController().enableContinuousInput(-2.5,//-drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond(),
    //     2.5);//drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond());
    getController().setTolerance(0.2);
  }

  @Override
  public void execute() {
    super.execute();

    SmartDashboard.putNumber("Target", getController().getGoal().position);
    SmartDashboard.putNumber("ACtual target", m_supp.getAsDouble());
    SmartDashboard.putNumber("current", getController().getPositionError());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
