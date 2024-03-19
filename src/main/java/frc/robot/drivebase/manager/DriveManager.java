package frc.robot.drivebase.manager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.drivebase.simpleBackForwardSwerve.SimpleBackForwardSwerveDriveTrain;
import frc.robot.drivebase.swerveDrive.SwerveDashboardWidgets;
import frc.robot.drivebase.swerveDrive.SwerveVariables;
import frc.robot.util.input.ControllerVariables;
import frc.robot.drivebase.swerveDrive.SwerveDriveTrain;

public class DriveManager {

    private final SwerveDriveTrain driveTrain;
    //private SimpleBackForwardSwerveDriveTrain simpleTrain;
    private final XboxController gamepad;

    private final SlewRateLimiter limiterX = new SlewRateLimiter(1);
    private final SlewRateLimiter limiterY = new SlewRateLimiter(1);
    private final SlewRateLimiter limiterRot = new SlewRateLimiter(1);

    private final EventLoop eventLoop = new EventLoop();

    public DriveManager(XboxController pad) {
        gamepad = pad;
        driveTrain = new SwerveDriveTrain();
        //simpleTrain = new SimpleBackForwardSwerveDriveTrain();

        gamepad.povUp(eventLoop).ifHigh(DriveManager::increaseDriveSpeed);
        gamepad.povDown(eventLoop).ifHigh(DriveManager::decreaseDriveSpeed);

        gamepad.povRight(eventLoop).ifHigh(DriveManager::increaseRotateSpeed);
        gamepad.povLeft(eventLoop).ifHigh(DriveManager::decreaseRotateSpeed);

        SwerveDashboardWidgets.initShuffleboardLayout();
    }

    public void poll(double periodSeconds) {
        eventLoop.poll();
        double x = limiterX
            .calculate(MathUtil.applyDeadband(gamepad.getLeftY(), ControllerVariables.LeftStickYDeadZone))
            * -SwerveVariables.getMaxSpeed();
        double y = limiterY
            .calculate(MathUtil.applyDeadband(gamepad.getLeftX(), ControllerVariables.LeftStickXDeadZone))
            * -SwerveVariables.getMaxSpeed();
        double rotation = limiterRot
            .calculate(MathUtil.applyDeadband(gamepad.getRightX(), ControllerVariables.RightStickXDeadZone))
            * SwerveVariables.maxAngularSpeed;

        SwerveDashboardWidgets.inputX.setDouble(gamepad.getLeftY());
        SwerveDashboardWidgets.inputY.setDouble(gamepad.getLeftX());
        SwerveDashboardWidgets.inputRot.setDouble(gamepad.getRightX());

        SwerveDashboardWidgets.outputX.setDouble(x);
        SwerveDashboardWidgets.outputY.setDouble(y);
        SwerveDashboardWidgets.outputRot.setDouble(rotation);

        driveTrain.updateWidget();
        //simpleTrain.updateWidget();

        if (Math.abs(x) < 0.01 && Math.abs(y) < 0.01 && Math.abs(rotation) < 0.01) {
            driveTrain.stop();
            //simpleTrain.stop();
        } else {
            //simpleTrain.drive(x,periodSeconds);
            driveTrain.drive(x, y, rotation, periodSeconds);
        }

    }

    public static void increaseDriveSpeed() {

        // SwerveVariables.maxSpeed += 0.1;
        SwerveDashboardWidgets.maxSpeedWidget.setDouble(
            SwerveVariables.getMaxSpeed() + 0.1);
    }

    public static void decreaseDriveSpeed() {
        SwerveDashboardWidgets.maxSpeedWidget.setDouble(
            Math.max(SwerveVariables.getMaxSpeed() - 0.1, 0));

    }

    public static void increaseRotateSpeed() {
        SwerveVariables.maxAngularSpeed += 0.05;
    }

    public static void decreaseRotateSpeed() {
        SwerveVariables.maxAngularSpeed = Math.max(SwerveVariables.maxAngularSpeed - 0.05, 0);

    }

}
