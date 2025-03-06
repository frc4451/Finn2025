package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;

public class ServoIOReal implements ServoIO {
    public Servo servo = new Servo(0);

    @Override
    public void updateInputs(ServoIOInputs inputs) {
        inputs.servoPositionAngle = servo.getAngle();
    }

    @Override
    public void setAngle(double angle) {
        servo.setAngle(angle);
    }
}
