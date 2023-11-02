package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollersubsystem.MoveSmartMotorControllerSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Shooter shooter, Supplier<Double> setpoint) {
        addRequirements(shooter);
        addCommands(new MoveSmartMotorControllerSubsystem(shooter, shooter.getPIDSettings(),
                shooter.getFeedForwardSettings(), UnifiedControlMode.VELOCITY, setpoint));
    }
}
