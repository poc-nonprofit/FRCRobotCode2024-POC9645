package frc.robot.drivebase.swerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Map;

public class SwerveModule {

    private Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;

    private final CANcoder cancoder;

    private final PIDController drivePID = new PIDController(0, 0, 0);
    private final PIDController rotatePID = new PIDController(0.1,0,0);

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward rotateFeedforward = new SimpleMotorFeedforward(0, 0);

    private final String identifier;
    private GenericEntry driveMotorWidget;
    private GenericEntry rotateMotorWidget;

    private GenericEntry encoderWidget;
    private GenericEntry rotationWidget;

    private GenericEntry driveEncoderWidget;

    private Rotation2d lastAngle;

    public SwerveModule(int driveMotorChannel, int rotateMotorChannel, int CANCoderChannel, Translation2d position,
                        String identifierStr) {
        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
        rotateMotor = new CANSparkMax(rotateMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();

        cancoder = new CANcoder(CANCoderChannel);

        identifier = identifierStr;


        initWidget();
    }

    private void initWidget() {
        driveMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Drive Motor " + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 0)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        rotateMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotate Motor " + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 1)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        encoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("CANCODER Value " + identifier, 0.0)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        rotationWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotation " + identifier, 0)
            .withPosition(2, 1)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

        driveEncoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("DRIVE ENCODER VALUE", 0)
            .getEntry();

    }

    public void updateWidget(double drive, double rotate) {
        encoderWidget.setDouble(cancoder.getPosition().getValue());
        driveMotorWidget.setDouble(drive);
        rotateMotorWidget.setDouble(rotate);
        rotationWidget
            .setDouble(Rotation2d.fromRotations(cancoder.getPosition().getValue()).getDegrees());
        driveEncoderWidget.setDouble(driveEncoder.getPosition());
    }

    public void updateWidget() {
        encoderWidget.setDouble(cancoder.getPosition().getValue());
        driveMotorWidget.setDouble(driveMotor.getAppliedOutput());
        rotateMotorWidget.setDouble(rotateMotor.getAppliedOutput());
        rotationWidget
            .setDouble(Rotation2d.fromRotations(cancoder.getPosition().getValue()).getDegrees());
        driveEncoderWidget.setDouble(driveEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(

        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition() / SwerveVariables.gearRatio
            * SwerveVariables.wheelRaduis * 2 * Math.PI/* add gear ratio calc */,
            new Rotation2d(cancoder.getPosition().getValue()));
    }

    public void setState(SwerveModuleState state) {
        Rotation2d currentRotation = new Rotation2d(cancoder.getPosition().getValue());

        SwerveModuleState newState = SwerveModuleState.optimize(state, currentRotation);

        state.speedMetersPerSecond *= state.angle.minus(currentRotation).getCos();

        Rotation2d newAngle = (Math.abs(newState.speedMetersPerSecond) <= (SwerveVariables.getMaxSpeed()/100) )
            ? lastAngle:
            newState.angle;

        /*double driveOutput = drivePID.calculate(driveEncoder.getVelocity(), newState.speedMetersPerSecond);
        double driveFeedForward = driveFeedforward.calculate(newState.speedMetersPerSecond);*/

        double rotateOutput = rotatePID.calculate(cancoder.getPosition().getValue(),newState.angle.getRotations());
        double rotateFeedForward = rotateFeedforward.calculate(rotatePID.getSetpoint());

        System.out.println(newState.angle.getDegrees());

        //double driveOutputValue = (driveOutput + driveFeedForward) / RobotController.getBatteryVoltage();
        //double driveOutputValue = newState.speedMetersPerSecond;
        double rotateOutputValue = (rotateOutput + rotateFeedForward) / RobotController.getBatteryVoltage();

        //updateWidget(driveOutputValue, rotateOutputValue);
        updateWidget(newState.speedMetersPerSecond, rotateOutputValue);

        driveMotor.set(newState.speedMetersPerSecond);
        rotateMotor.set(rotateOutputValue);
        /*driveMotor.set(driveOutputValue);
        rotateMotor.set(rotateOutputValue);*/
        //rotateMotor.set(newState.angle.getRadians() /10);


    }

    public void stop() {

        driveMotorWidget.setDouble(0);
        rotateMotorWidget.setDouble(0);

        if (driveMotor != null)
            driveMotor.set(0);
        if (rotateMotor != null)
            rotateMotor.set(0);
    }
}
