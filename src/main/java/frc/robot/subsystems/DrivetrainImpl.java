package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.util.TheBetterLimelight;

import java.util.function.Supplier;

public class DrivetrainImpl extends DashboardedSubsystem implements Drivetrain {
    // https://cdn.discordapp.com/attachments/927272978356510721/1167115100117807264/uwuyd99s0cub1.png?ex=654cf3a3&is=653a7ea3&hm=4fd387e2c5dbac2377e7a6c69bceb3218edb077aaeb6685f592d95a89ef7923c&
    public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(0, 0); // @TODO find positions
    public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d BACK_LEFT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d BACK_RIGHT_WHEEL_POSITION = new Translation2d(0, 0);
    public static final Translation2d CENTER_OF_ROBOT = new Translation2d(
            (FRONT_LEFT_WHEEL_POSITION.getX() + BACK_RIGHT_WHEEL_POSITION.getX()) / 2,
            (FRONT_LEFT_WHEEL_POSITION.getY() + BACK_RIGHT_WHEEL_POSITION.getY()) / 2);

    public static final double MAX_SPEED_METERS_PER_SECONDS = 0;

    private static final String NAMESPACE_NAME = "drivetrain";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Gyro gyro;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final TheBetterLimelight limelight;
    private NetworkTableValue targetingLatency;
    private NetworkTableValue captureLatency;

    private Pose2d estimatedVisionPose;

    public DrivetrainImpl(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight,
                          Gyro gyro) {
        super(NAMESPACE_NAME);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = new SwerveDriveKinematics(FRONT_LEFT_WHEEL_POSITION, FRONT_RIGHT_WHEEL_POSITION,
                BACK_LEFT_WHEEL_POSITION, BACK_RIGHT_WHEEL_POSITION);
        this.gyro = gyro;
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new Pose2d()); // @TODO check start position
        limelight = new TheBetterLimelight();
    }

    @Override
    public void periodic() {
        super.periodic();
        estimatedVisionPose = limelight.getRobotPose().toPose2d();
        targetingLatency = limelight.getValue("tl");
        captureLatency = limelight.getValue("cl");
        if (estimatedVisionPose != null) {
            poseEstimator.addVisionMeasurement(estimatedVisionPose,
                    Timer.getFPGATimestamp() - (targetingLatency.getDouble() / 1000.0) -
                            (captureLatency.getDouble() / 1000.0));
        }
        poseEstimator.update(gyro.getRotation2d(), new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });
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
