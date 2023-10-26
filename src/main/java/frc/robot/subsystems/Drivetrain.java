package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.util.function.Supplier;

public class Drivetrain extends DashboardedSubsystem {

    public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d BACK_LEFT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d BACK_RIGHT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d CENTER_OF_ROBOT = new Translation2d(
            (FRONT_LEFT_WHEEL_POSITION.getX() + BACK_RIGHT_WHEEL_POSITION.getX()) / 2,
            (FRONT_LEFT_WHEEL_POSITION.getY() + BACK_RIGHT_WHEEL_POSITION.getY()) / 2);

    public static final double MAX_SPEED_METERS_PER_SECONDS = 0;

    private static final String namespaceName = "drivetrain";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;

    private final Gyro gyro;

    public Drivetrain(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight,
                      Gyro gyro) {
        super(namespaceName);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION, FRONT_RIGHT_WHEEL_POSITION,
                BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        this.gyro = gyro;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed,
                      boolean fieldRelative, boolean usePID) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed.get(), ySpeed.get(),
                    rotationSpeed.get(), gyro.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed.get(), ySpeed.get(), rotationSpeed.get());
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds, CENTER_OF_ROBOT);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECONDS);
        frontLeft.set(states[0], usePID);
        frontRight.set(states[1], usePID);
        backLeft.set(states[2], usePID);
        backRight.set(states[3], usePID);
    }

    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void configureDashboard() {
    }
}
