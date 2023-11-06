package frc.robot;

public enum ShootingConfig {

    I_NEED_TO_PUT_SOMETHING_HERE_SO_THE_CODE_WILL_COMPILE_SO_I_HAVE_DECIDED_TO_TYPE_A_COMICALLY_LONG_NAME(0, 0, 0, 0 ,0);

    public final double x;
    public final double y;
    public final double robotAngle;
    public final double shooterVelocity;
    public final double hoodPosition;

    ShootingConfig(double x, double y, double robotAngle, double shooterVelocity, double hoodPosition) {
        this.x = x;
        this.y = y;
        this.robotAngle = robotAngle;
        this.shooterVelocity = shooterVelocity;
        this.hoodPosition = hoodPosition;
    }
}
