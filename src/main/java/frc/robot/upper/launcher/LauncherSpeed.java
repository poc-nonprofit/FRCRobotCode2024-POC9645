package frc.robot.upper.launcher;

public class LauncherSpeed {
    public static float launcherMaxSpeed = 0.4f;

    public static void increaseLauncherMaxSpeed(){
        launcherMaxSpeed = Math.min(launcherMaxSpeed + 0.1f , 1.0f);
    }
    public static void decreaseLauncherMaxSpeed(){
        launcherMaxSpeed = Math.max(launcherMaxSpeed - 0.1f , 1.0f);
    }
    public static void changeLauncherMaxSpeed(float diff){
        launcherMaxSpeed = Math.max(Math.min(launcherMaxSpeed + diff , 1.0f), 0f);
    }
}
