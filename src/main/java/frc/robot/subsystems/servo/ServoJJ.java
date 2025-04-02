package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoJJ extends SubsystemBase {
    private Servo servo;

    public ServoJJ(int channel) {
        servo = new Servo(channel);
    }

    public Command setAngle(int ang) {
        return runOnce(() -> servo.setAngle(ang));
    }

    public Command setAngle(double angl) {
        return runOnce(() -> servo.set(angl * 1.4446));
    }
}
