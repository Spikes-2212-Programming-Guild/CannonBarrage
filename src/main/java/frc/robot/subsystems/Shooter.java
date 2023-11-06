package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public interface Shooter extends SmartMotorControllerGenericSubsystem {

    void setVelocity(double velocity);
    double getVelocity();
    void stop();
    PIDSettings getPIDSettings();
    FeedForwardSettings getFeedForwardSettings();
}
