// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.auton.CarsonVPath;
import frc.robot.commands.auton.EpicPath;
import frc.robot.commands.auton.JPath;
import frc.robot.commands.auton.JPath1;
import frc.robot.commands.auton.JPath2;
import frc.robot.commands.auton.NickPath;
import frc.robot.commands.auton.RotateDrivetrainToAngle;
import frc.robot.commands.auton.RotateDrivetrainToTargetPosition;
import frc.robot.commands.auton.SamPath;
import frc.robot.commands.auton.TestPath;
import frc.robot.commands.auton.TwoPieceAcquirePiece;
import frc.robot.commands.auton.TwoPieceDriveUp;
import frc.robot.commands.auton.TwoPieceScorePiece;
import frc.robot.subsystems.CIMDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.FalconDrivetrain;
import frc.robot.subsystems.NEODrivetrain;
import frc.robot.subsystems.OctavianSwerveDrivetrain;
import frc.robot.subsystems.PracticeSwerveDrivetrain;
import frc.robot.subsystems.SixNEODrivetrain;
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
    // private OctavianSwerveDrivetrain m_drive = OctavianSwerveDrivetrain.getInstance();
    private SwerveDrivetrain m_drive = SwerveDrivetrain.getInstance();
    // private PracticeSwerveDrivetrain m_drive = PracticeSwerveDrivetrain.getInstance();
    
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
                        -speedScaledDriverLeftY(),
                        speedScaledDriverLeftX(),
                        speedScaledDriverRightX(),
                        true),
                        m_drive));

        m_driverController.start.onTrue(new InstantCommand(m_drive::zero));
        m_driverController.a.onTrue(new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), m_drive, false));
        m_driverController.b.onTrue(new RotateDrivetrainToTargetPosition(Distance.fromInches(324.), Distance.fromInches(162.), m_drive).withTimeout(2.0));
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
        _autonChooser.addOption("Carson V Path", new CarsonVPath(m_drive));
        _autonChooser.addOption("Sam Path", new SamPath(m_drive));
        _autonChooser.addOption("Nick Path", new NickPath(m_drive));
        _autonChooser.addOption("j path 1", new JPath1(m_drive));
        _autonChooser.addOption("j path 2", new JPath2(m_drive));
        _autonChooser.addOption("J Path", new JPath(m_drive));
        _autonChooser.addOption("Two Piece Drive Up", new TwoPieceDriveUp(m_drive));
        _autonChooser.addOption("Two Piece Acquire Piece", new TwoPieceAcquirePiece(m_drive));
        _autonChooser.addOption("Two Piece Score Piece", new TwoPieceScorePiece(m_drive));

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
