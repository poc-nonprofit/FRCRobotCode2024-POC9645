package frc.robot.drivebase.swerveDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.drivebase.swerveDrive.SwerveDashboardWidgets.chassisAngle;

public class SwerveDriveTrain {

    private final SwerveModule swerveFR = new SwerveModule(3, 4, 12, SwerveVariables.moduleLocations.FR.getValue(), "Front Right");
    private final SwerveModule swerveRR = new SwerveModule(5, 6, 13, SwerveVariables.moduleLocations.RR.getValue(), "Rear Right");
    private final SwerveModule swerveRL = new SwerveModule(7, 8, 14, SwerveVariables.moduleLocations.RL.getValue(), "Rear Left");
    private final SwerveModule swerveFL = new SwerveModule(1, 2, 11, SwerveVariables.moduleLocations.FL.getValue(), "Front Left");

    private final AHRS gyroSensor = new AHRS(SPI.Port.kMXP);

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveDriveState", SwerveModuleState.struct).publish();

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            SwerveVariables.moduleLocations.FL.getValue(),
            SwerveVariables.moduleLocations.FR.getValue(),
            SwerveVariables.moduleLocations.RL.getValue(),
            SwerveVariables.moduleLocations.RR.getValue()
        );

    private final SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyroSensor.getAngle()),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition(),
            }
        );

    public SwerveDriveTrain() {
        gyroSensor.reset();
        //gyroSensor.calibrate();
    }

    public void drive(double x, double y, double rotation, double periodSeconds) {
        SwerveModuleState[] state = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                /*SwerveVariables.fieldOriented*/ SwerveVariables.getFieldOriented()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(x,y,rotation,Rotation2d.fromDegrees(gyroSensor.getAngle()))
                    : new ChassisSpeeds(x,y,rotation),periodSeconds
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(state,SwerveVariables.maxSpeed);

        publisher.set(state);


        swerveFL.setState(state[0]);
        swerveFR.setState(state[1]);
        swerveRL.setState(state[2]);
        swerveRR.setState(state[3]);

    }

    public void stop(){
        swerveFL.stop();
        swerveFR.stop();
        swerveRL.stop();
        swerveRR.stop();
    }

    public void updateWidget(){
        swerveFL.updateWidget();
        swerveFR.updateWidget();
        swerveRL.updateWidget();
        swerveRR.updateWidget();
        chassisAngle.setDouble(gyroSensor.getAngle());
    }


    public void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(gyroSensor.getAngle()),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition()
            });
    }

}
