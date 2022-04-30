// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.auton.EpicPath;
import frc.robot.commands.auton.RotateDrivetrainToAngle;
import frc.robot.commands.auton.TestPath;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class RobotContainer {
    private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.DRIVER);

    private Drivetrain m_drive = Drivetrain.getInstance();
    private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();

    private static RobotContainer _instance = new RobotContainer();

    public RobotContainer() {
        configureButtonBindings();
        initAutonChooser();
    }

    public void configureButtonBindings() {
        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        speedScaledDriverLeftY(),
                        0.,
                        speedScaledDriverRightX()),
                        m_drive));

        m_driverController.start.whenPressed(m_drive::zero);
        m_driverController.a.whenPressed(new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), m_drive));
    }

    public double speedScaledDriverLeftY() {
        return Util.speedScale(m_driverController.getLeftYAxis(),
            DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger());
    }

    public double speedScaledDriverRightX() {
        return -Util.speedScale(m_driverController.getRightXAxis(),
            DriveConstants.SPEED_SCALE,
            m_driverController.getRightTrigger());
    }

    private void initAutonChooser() {
        _autonChooser.setDefaultOption("Epic Path", new EpicPath(m_drive));
        _autonChooser.addOption("Test Path", new TestPath(m_drive));

        SmartDashboard.putData("Auton Chooser", _autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_drive.resetOdometry(_autonChooser.getSelected().getInitialPose());
        return _autonChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return _instance;
    }
}
