package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;

public interface Hood extends SmartMotorControllerGenericSubsystem {

    PIDSettings getPIDSettings();
    FeedForwardSettings getFeedForwardSettings();
    TrapezoidProfileSettings getTrapezoidProfileSettings();
}
