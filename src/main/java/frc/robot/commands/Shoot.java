package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShootingConfig;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {

    public Shoot(Shooter shooter, Hood hood, ShootingConfig shootingConfig) {
        addRequirements(shooter, hood);
        addCommands(new MoveSmartMotorControllerSubsystemTrapezically(hood, hood.getPIDSettings(),
                        hood.getFeedForwardSettings(),
                        () -> shootingConfig.hoodPosition, hood.getTrapezoidProfileSettings()),
                new MoveSmartMotorControllerGenericSubsystem(shooter, shooter.getPIDSettings(),
                        shooter.getFeedForwardSettings(), UnifiedControlMode.VELOCITY,
                        () -> shootingConfig.shooterVelocity));
    }
}
