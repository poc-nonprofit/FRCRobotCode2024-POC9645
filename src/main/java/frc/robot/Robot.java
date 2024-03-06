package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.upper.launcher.LauncherSpeed;


public class Robot extends TimedRobot {

    private final EventLoop eventLoop = new EventLoop();

    private final XboxController gamepad = new XboxController(0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {

    }
    @Override
    public void robotPeriodic() {
        eventLoop.poll();
    }


    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        gamepad.rightBumper(eventLoop).ifHigh(LauncherSpeed::increaseLauncherMaxSpeed);
        gamepad.leftBumper(eventLoop).ifHigh(LauncherSpeed::decreaseLauncherMaxSpeed);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

}
