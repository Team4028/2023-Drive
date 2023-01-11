// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.CTREPhysicsSim;
import frc.robot.utilities.log.BeakLogger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static BeakLogger m_currentLogger;
    private static BeakLogger m_autonLogger;
    private static BeakLogger m_teleopLogger;
    private static BeakLogger m_disabledLogger;
    private static BeakLogger m_testLogger;

    // private String m_logFileFolder = "testLogs";
    private Command m_autonCommand;
    private RobotContainer m_robotContainer;

    public static BeakLogger getCurrentLogger() {
        return m_currentLogger;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_robotContainer = RobotContainer.getInstance();
        // Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        // try {
        //     m_autonLogger = new BeakLogger(m_logFileFolder, "auton");
        //     m_teleopLogger = new BeakLogger(m_logFileFolder, "teleop");
        //     m_disabledLogger = new BeakLogger(m_logFileFolder, "disabled");
        //     m_testLogger = new BeakLogger(m_logFileFolder, "test");
        // } catch (IOException error) {
        //     System.err.println("Failed to open log file(s): " + error.getMessage());
        // }

        m_currentLogger = m_disabledLogger;

        // m_robotContainer.logAllConfigs();
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_currentLogger = m_autonLogger;

        m_autonCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonCommand != null) {
            m_autonCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        m_currentLogger = m_teleopLogger;
        
        if (m_autonCommand != null) {
            m_autonCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        m_currentLogger = m_disabledLogger;
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        m_currentLogger = m_testLogger;
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        CTREPhysicsSim.getInstance().run();
        REVPhysicsSim.getInstance().run();
    }
}
