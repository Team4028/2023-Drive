// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.drive.BeakDrivetrain;
import frc.robot.utilities.units.Distance;

// TODO: This is broken.
public class CorrectSkewAndAngle extends CommandBase {
  private final ProfiledPIDController m_thetaController;
  private final ProfiledPIDController m_skewController;

  private final Distance m_distance;
  private final BeakDrivetrain m_drive;
  private final Limelight m_limelight;
  /** Creates a new MoveToTargetDistance. */
  public CorrectSkewAndAngle(Distance distance, BeakDrivetrain drivetrain, Limelight limelight) {
    m_distance = distance;
    m_drive = drivetrain;
    m_limelight = limelight;

    System.out.println(distance.getAsMeters());

    m_thetaController = new ProfiledPIDController(
      4.4,
      0.015,
      0.4,
      new TrapezoidProfile.Constraints(
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond(),
                drivetrain.getPhysics().maxAngularVelocity.getAsRadiansPerSecond())); // TODO: getThetaConstraints()
    m_thetaController.setGoal(0.);
    m_thetaController.setTolerance(1.0);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_skewController = new ProfiledPIDController(
      0.05,
      0.,
      0.0,
      new TrapezoidProfile.Constraints(
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25,
                drivetrain.getPhysics().maxVelocity.getAsMetersPerSecond() * 0.25));
    m_skewController.setGoal(0.);
    m_skewController.setTolerance(0.2);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = m_thetaController.calculate(Units.degreesToRadians(m_limelight.getX()), 0.);
    State thetaSetpoint = m_thetaController.getSetpoint();

    double skew = m_limelight.getSkew();
    skew = m_limelight.getX() > 0 ? skew - 90 : skew;

    double skewOutput = m_skewController.calculate(skew, 0.);
    State skewSetpoint = m_skewController.getSetpoint();

    SmartDashboard.putNumber("output", skewOutput + skewSetpoint.velocity);

    m_drive.drive( // TODO: PLEASE DRIVERAW
      0.,
      -(skewOutput + skewSetpoint.velocity) / m_drive.getPhysics().maxVelocity.getAsMetersPerSecond(),
      (thetaOutput + thetaSetpoint.velocity) / m_drive.getPhysics().maxAngularVelocity.getAsRadiansPerSecond(),
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(
      0.,
      0.,
      0.
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_thetaController.atGoal() && m_skewController.atGoal();
  }
}
