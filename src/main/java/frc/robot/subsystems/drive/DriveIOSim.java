package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {

    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide,
            KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private PIDController leftPID = new PIDController(DriveConstants.kSimKp, DriveConstants.kSimKi,
            DriveConstants.kSimKd);
    private PIDController rightPID = new PIDController(DriveConstants.kSimKp, DriveConstants.kSimKi,
            DriveConstants.kSimKd);
    private double leftFFVolts = 0.0;
    private double rightFFVolts = 0.0;

    private boolean closedLoop = false;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        if (closedLoop) {
            double left = leftFFVolts
                    + leftPID.calculate(sim.getLeftVelocityMetersPerSecond() * DriveConstants.kWheelRadiusMeters);
            double right = rightFFVolts + rightPID
                    .calculate(sim.getRightVelocityMetersPerSecond() * DriveConstants.kWheelRadiusMeters);

            runVolts(left, right);
        }

        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
        sim.update(0.02);

        inputs.leftPositionRad = sim.getLeftPositionMeters() / DriveConstants.kWheelRadiusMeters;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters;
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = new double[] { sim.getLeftCurrentDrawAmps() };

        inputs.rightPositionRad = sim.getRightPositionMeters() / DriveConstants.kWheelRadiusMeters;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = new double[] { sim.getRightCurrentDrawAmps() };
    }

    private void runVolts(double leftVolts, double rightVolts) {
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        closedLoop = false;
        runVolts(leftVolts, rightVolts);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        closedLoop = true;
        this.leftFFVolts = leftFFVolts;
        this.rightFFVolts = rightFFVolts;
        leftPID.setSetpoint(leftRadPerSec);
        rightPID.setSetpoint(rightRadPerSec);
    }
}
