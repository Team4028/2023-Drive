// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.auton.MoveDrivetrainToTargetDistance;
import frc.robot.commands.auton.CorrectSkewAndAngle;
import frc.robot.commands.auton.RotateDrivetrainByLimelightAngle;
import frc.robot.commands.auton.RotateDrivetrainToAngle;
import frc.robot.commands.auton.RotateDrivetrainToTargetPosition;
import frc.robot.commands.auton.paths.EpicPath;
import frc.robot.commands.auton.paths.JPath;
import frc.robot.commands.auton.paths.TestPath;
import frc.robot.subsystems.CIMDrivetrain;
import frc.robot.subsystems.EpicSwerveDrivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Mk2SwerveDrivetrain;
import frc.robot.subsystems.NEODrivetrain;
import frc.robot.subsystems.OctavianSwerveDrivetrain;
import frc.robot.subsystems.SixNEODrivetrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utilities.BeakXBoxController;
import frc.robot.utilities.Util;
import frc.robot.utilities.units.Distance;

/** Add your docs here. */
public class RobotContainer {
    private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.DRIVER);

    // private NEODrivetrain m_drive = NEODrivetrain.getInstance();
    // private SixNEODrivetrain m_drive = SixNEODrivetrain.getInstance();
    // private CIMDrivetrain m_drive = CIMDrivetrain.getInstance();
    // private FalconDrivetrain m_drive = FalconDrivetrain.getInstance();
    // private Mk2SwerveDrivetrain m_drive = Mk2SwerveDrivetrain.getInstance();
    // private OctavianSwerveDrivetrain m_drive = OctavianSwerveDrivetrain.getInstance();
    // private SwerveDrivetrain m_drive = SwerveDrivetrain.getInstance();
    private EpicSwerveDrivetrain m_drive = EpicSwerveDrivetrain.getInstance();

    private Limelight m_limelight = Limelight.getInstance();
    
    private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();

    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(4.0);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4.0);
    
    private static RobotContainer _instance = new RobotContainer();

    public RobotContainer() {
        configureButtonBindings();
        initAutonChooser();
    }

    public void logAllConfigs() {
        // m_drive.logConfigData();
    }

    public void configureButtonBindings() {
        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        speedScaledDriverLeftY(),
                        speedScaledDriverLeftX(),
                        speedScaledDriverRightX(),
                        true),
                        m_drive));

        m_driverController.start.whenPressed(m_drive::zero);
        m_driverController.a.whenPressed(new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), m_drive, false));
        m_driverController.b.whenPressed(new RotateDrivetrainToTargetPosition(Distance.fromInches(324.), Distance.fromInches(162.), m_drive).withTimeout(2.0));
        m_driverController.x.whenPressed(new RotateDrivetrainByLimelightAngle(m_limelight, m_drive).withTimeout(2.0));
        // m_driverController.y.whenPressed(new MoveDrivetrainToTargetDistance(Distance.fromFeet(19.), m_limelight, m_drive).withTimeout(999999.0));
        m_driverController.y.whenPressed(new CorrectSkewAndAngle(Distance.fromFeet(2.), m_drive, m_limelight).withTimeout(2.0));
        m_driverController.back.whenPressed(new MoveDrivetrainToTargetDistance(Distance.fromFeet(2.), m_limelight, m_drive).withTimeout(2.0));
        // -1.75
    }

    public double speedScaledDriverLeftY() {
        return m_yLimiter.calculate(Util.speedScale(m_driverController.getLeftYAxis(),
                DriveConstants.SPEED_SCALE,
                m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverRightX() {
        return m_rotLimiter.calculate(-Util.speedScale(m_driverController.getRightXAxis(),
                DriveConstants.SPEED_SCALE,
                m_driverController.getRightTrigger()));
    }

    public double speedScaledDriverLeftX() {
        return m_xLimiter.calculate(-Util.speedScale(m_driverController.getLeftXAxis(),
                DriveConstants.SPEED_SCALE,
                m_driverController.getRightTrigger()));
    }

    private void initAutonChooser() {
        _autonChooser.setDefaultOption("Epic Path", new EpicPath(m_drive));
        _autonChooser.addOption("Test Path", new TestPath(m_drive));
        _autonChooser.addOption("J path", new JPath(m_limelight, m_drive));

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
