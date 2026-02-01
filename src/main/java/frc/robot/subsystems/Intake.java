package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake {

    double speed = 0.5;
    // 1. Declare the motor
    private final TalonFX motor = new TalonFX(10); // Replace 10 with your CAN ID
    private final DutyCycleOut request = new DutyCycleOut(0);
        // 2. Configure motor
    private final TalonFXConfiguration configs = new TalonFXConfiguration();
        // Optional: Set current limits to prevent burning out motors (e.g., 40A)
    // 3. Set speed (percentage -1.0 to 1.0)

    public void IntakeEnable(double speed) {
        motor.setControl(request.withOutput(-speed));
    }

    public void Score(double speed) {
        motor.setControl(request.withOutput(speed));
    }

}