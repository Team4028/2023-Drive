// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.drive.BeakDrivetrain;

// TODO: Need to try this, and see if it works better than a ProfiledPIDCommand
public class LimelightRotationTest extends CommandBase {
  private ProfiledPIDController m_controller;

  private BeakDrivetrain m_drive;
  private Limelight m_limelight;
  /** Creates a new LimelightRotationTest. */
  public LimelightRotationTest(BeakDrivetrain drivetrain, Limelight limelight) {
    m_drive = drivetrain;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
