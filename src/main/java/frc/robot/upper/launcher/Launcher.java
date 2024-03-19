package frc.robot.upper.launcher;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.upper.launcher.LauncherSpeed.launcherMaxSpeed;

public class Launcher {

    private XboxController gamepad;
    private final EventLoop eventLoop = new EventLoop();

    private CANSparkMax motor1;
    private CANSparkMax motor2;

    private final SlewRateLimiter limiter1 = new SlewRateLimiter(0.1);
    private final SlewRateLimiter limiter2 = new SlewRateLimiter(0.1);

    public Launcher(XboxController pad) {
        motor1 = new CANSparkMax(/* 9 */20, MotorType.kBrushless);
        motor2 = new CANSparkMax(/* 8 */21, MotorType.kBrushless);
        motor2.setInverted(true);
        gamepad = pad;
        gamepad.start(eventLoop).ifHigh(Launcher::increaseLauncherMaxSpeed);
        gamepad.back(eventLoop).ifHigh(Launcher::decreaseLauncherMaxSpeed);
    }

    public void poll() {
        eventLoop.poll();
        if (gamepad.getRightTriggerAxis() != 0) {
            motor1.set(limiter1.calculate(gamepad.getRightTriggerAxis() * launcherMaxSpeed));
            motor2.set(limiter2.calculate(gamepad.getRightTriggerAxis() * launcherMaxSpeed));
        } else if (gamepad.getRightBumperPressed()) {
            motor1.set(limiter1.calculate(launcherMaxSpeed));
            motor2.set(limiter1.calculate(launcherMaxSpeed));
        }
    }

    public static void increaseLauncherMaxSpeed() {
        launcherMaxSpeed = Math.min(launcherMaxSpeed + 0.1, 1.0);
    }

    public static void decreaseLauncherMaxSpeed() {
        launcherMaxSpeed = Math.max(launcherMaxSpeed - 0.1, 0.0);
    }

    public static void changeLauncherMaxSpeed(float diff) {
        launcherMaxSpeed = Math.max(Math.min(launcherMaxSpeed + diff, 1.0), 0);
    }
}
