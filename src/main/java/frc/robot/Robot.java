package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivebase.manager.DriveManager;

import frc.robot.upper.launcher.Launcher;


public class Robot extends TimedRobot {

    private final EventLoop eventLoop = new EventLoop();

    private final XboxController gamepad = new XboxController(0);

    private DriveManager driveManager;
    private Launcher launcher;

    @Override
    public void robotInit() {
        driveManager = new DriveManager(gamepad);
        launcher = new Launcher(gamepad);
    }

    @Override
    public void robotPeriodic() {
        eventLoop.poll();
        driveManager.poll(getPeriod());
        launcher.poll();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

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
