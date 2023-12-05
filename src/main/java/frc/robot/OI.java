package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModuleHolder;

public class OI /*GEVALD*/{

    private PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    public OI() {
        ps.getCrossButton().onTrue(new InstantCommand(new SwerveModuleHolder().getFrontLeft()::configureRelativeEncoder));
    }
}
