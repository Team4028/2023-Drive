// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateDrivetrainToAngle extends ProfiledPIDCommand {
    /** Creates a new RotateDrivetrainByAngle. */
    public RotateDrivetrainToAngle(Rotation2d goal) {
        super(
                // The ProfiledPIDController used by the command
                AutonConstants.THETA_CONTROLLER,
                // This should return the measurement
                () -> Drivetrain.getInstance().getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> goal.getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    Drivetrain.getInstance().drive(
                            0.,
                            0.,
                            output + setpoint.velocity);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(Drivetrain.getInstance());
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(0.5));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
