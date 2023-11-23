package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleImpl extends DashboardedSubsystem implements SwerveModule {

    public static final double FRONT_LEFT_OFFSET = 0;
    public static final double FRONT_RIGHT_OFFSET = 0;
    public static final double BACK_LEFT_OFFSET = 0;
    public static final double BACK_RIGHT_OFFSET = 0;

    private static final int PID_SLOT = 0;
    private static final double STEERING_GEAR_RATIO = 12.8;
    private static final double DRIVING_GEAR_RATIO = 6.12;
    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_METERS * Math.PI;

    private final CANSparkMax driveController;
    private final CANSparkMax turnController;
    private final CANCoder absoluteEncoder;
    private final double offset;
    private final FeedForwardController driveFeedForwardController;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private double lastAngle;

    public SwerveModuleImpl(String namespaceName, CANSparkMax driveController, CANSparkMax turnController,
                            CANCoder absoluteEncoder, double offset, FeedForwardSettings driveFeedForwardSettings,
                            PIDSettings drivePIDSettings, PIDSettings turnPIDSettings) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteEncoder = absoluteEncoder;
        this.offset = offset;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        driveEncoder = driveController.getEncoder();
        turnEncoder = turnController.getEncoder();
        lastAngle = getAbsoluteAngle();
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureRelativeEncoder();
        configureDashboard();
    }

    @Override
    public double getAbsoluteAngle() {
        return absoluteEncoder.getAbsolutePosition();
    }

    private double getRelativeAngle() {
        return turnEncoder.getPosition();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    @Override
    public void set(SwerveModuleState state, boolean usePID) {
        double angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainImpl.MAX_SPEED * 0.01))
                ? lastAngle : state.angle.getDegrees();
        state = optimize(state, Rotation2d.fromDegrees(getRelativeAngle()));
        setAngle(angle);
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    private void configureDriveController() {
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
        driveEncoder.setPositionConversionFactor((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS);
        driveEncoder.setVelocityConversionFactor(
                ((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS) / 60);
    }

    private void configureTurnController() {
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
    }

    private void configureAbsoluteEncoder() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configMagnetOffset(offset);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    private void configureRelativeEncoder() {
        turnController.getEncoder().setPositionConversionFactor((1 / STEERING_GEAR_RATIO) * 360);
        turnController.getEncoder().setPosition(getAbsoluteAngle());
    }

    //angle between 0 and 360
    private void setAngle(double angle) {
        configureTurnController();
        turnController.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT);
    }

    //speed - m/s
    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            driveFeedForwardController.setGains(driveFeedForwardSettings.getkS(), driveFeedForwardSettings.getkV(),
                    driveFeedForwardSettings.getkA(), driveFeedForwardSettings.getkG());
            double feedForward = driveFeedForwardController.calculate(speed);
            driveController.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity, PID_SLOT,
                    feedForward);
        } else driveController.set(speed / DrivetrainImpl.MAX_SPEED);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     * Credit to team #364
     *
     * @param desiredState the desired state
     * @param currentAngle the current module angle
     */
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Takes the module's angle and the desired angle, and returns it in within the range of 0 to 360.
     * Credit to team #364.
     *
     * @param scopeReference current angle
     * @param newAngle       target angle
     * @return closest angle within scope
     */
    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", driveEncoder.getVelocity());
        namespace.putNumber("distance", driveEncoder.getPosition());
        namespace.putNumber("angle", getAbsoluteAngle());
    }
}
