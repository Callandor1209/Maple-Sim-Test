package frc.robot.util;

import java.util.Optional;
import java.util.concurrent.BlockingQueue;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants;

public class MLClass {
    double[][] data;
    Command[] keys;
    double biasTime =3;
    double biasOnHomeSide = 1;
    double biasVisionDetectsCargo = 1;
    double biasCargoScored = 5;

    
    public void train(){
        //really advanced training method
        data = Constants.MLData.DATA;
        keys = Constants.MLData.KEYS;
    }

    public Command calculateNearestNeighbor(SwerveDriveSimulation driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, boolean isBlue){
        double time = Robot.matchTimer.get() /60;

        double onHomeSide;
        if(driveSimulation.getSimulatedDriveTrainPose().getX() > 8.15){
            onHomeSide = 0;
        }
        else onHomeSide = 1;

        double visionDetectsCargo;
        if(visionSubsystem.returnArea() > 0){
            visionDetectsCargo = 1;
        }
        else visionDetectsCargo = 0;

        double cargoScored = SimulatedArena.getInstance().getScore(isBlue);

        double[] currentConditions = {time,onHomeSide,visionDetectsCargo,cargoScored};

 

        int closestNeighbor =0;
        double closestNeighborDistance = 100000;
        for(int i = 0; i < data.length; i++){
            double distance = distanceCalculations(currentConditions, data[i]);
            if(distance < closestNeighborDistance){
                closestNeighbor = i;
                closestNeighborDistance = distance;
            }
        }
        return keys[closestNeighbor];
    }

    public double distanceCalculations(double[] inputDoubleArray, double[] compareDoubleArray){
        double sum = 0;
        for(int i = 0; i < inputDoubleArray.length; i++){
           double difference = inputDoubleArray[i] - compareDoubleArray[i];
            sum = sum + difference * difference;
        }
        sum = Math.sqrt(sum);
        return sum;
    }
}