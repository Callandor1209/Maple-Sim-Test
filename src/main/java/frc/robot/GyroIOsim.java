package frc.robot;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.GyroIO;

public class GyroIOsim implements GyroIO{
    private final GyroSimulation gyroSimulation;
    public GyroIOsim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }

    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
}
