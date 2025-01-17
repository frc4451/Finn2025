package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
    
    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private PIDController leftPID = new PIDController(DriveConstants.kSimKp, DriveConstants.kSimKi, DriveConstants.kSimKd);
    private PIDController rightPID = new PIDController(DriveConstants.kSimKp, DriveConstants.kSimKi, DriveConstants.kSimKd);

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        
        sim.setInputs(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0), MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
        sim.update(0.02);

        inputs.leftPositionRad = sim.getLeftPositionMeters() / DriveConstants.kWheelRadiusMeters;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters;
        inputs.leftAppliedVolts = leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters);
        inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};
        
        inputs.rightPositionRad = sim.getRightPositionMeters() / DriveConstants.kWheelRadiusMeters;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters;
        inputs.rightAppliedVolts = rightPID.calculate(sim.getRightVelocityMetersPerSecond() / DriveConstants.kWheelRadiusMeters);
        inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftAppliedVolts = leftVolts;
        rightAppliedVolts = rightVolts;
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec) {
        leftPID.setSetpoint(leftRadPerSec);
        rightPID.setSetpoint(rightRadPerSec);
    }
}
