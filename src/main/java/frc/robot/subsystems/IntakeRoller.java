package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.function.Supplier;

public abstract class IntakeRoller extends GenericSubsystem {

    public IntakeRoller(String namespaceName) {
        super(namespaceName);

    }
}
