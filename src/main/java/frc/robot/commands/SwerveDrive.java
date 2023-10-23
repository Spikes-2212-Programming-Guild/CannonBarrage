package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.function.Supplier;
// https://www.youtube.com/watch?v=9BaWt_BrYUg
public class SwerveDrive extends CommandBase {

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> rotationSpeed;

    public SwerveDrive(SwerveDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
                       Supplier<Double> rotationSpeed) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
    }

    @Override
    public void execute() {
        drivetrain.drive(xSpeed, ySpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
