package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
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
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings, FeedForwardController.DEFAULT_PERIOD);
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
        return turnController.getEncoder().getPosition();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveController.getEncoder().getPosition(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    @Override
    public void set(SwerveModuleState state, boolean usePID) {
        state = optimize(state, Rotation2d.fromDegrees(getRelativeAngle()));
        setAngle(state.angle.getDegrees());
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    private void configureDriveController() {
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
        driveController.getEncoder().setPositionConversionFactor((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS);
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
        turnController.getEncoder().setPositionConversionFactor((1 / STEERING_GEAR_RATIO) * 180);
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
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
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
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
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
    }
}
