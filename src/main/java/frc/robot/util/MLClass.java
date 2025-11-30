package frc.robot.util;

import java.util.Optional;
import java.util.concurrent.BlockingQueue;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class MLClass {
    double[][] data;
    Command[] keys;
    double biasTime =3;
    double biasOppScore = 0;
    double biasMyScore = 0;
    public void train(){
        //really advanced training method
        data = Constants.MLData.DATA;
        keys = Constants.MLData.KEYS;
    }

    //only add in the optional value for testing and stuff
    public Command calculateNearestNeighbor(Optional<double[]> inputArray){
        double timer = Timer.getMatchTime() /60;
        double [] currentConditions;
        if(inputArray.isEmpty()){
            double blueScore = SimulatedArena.getInstance().getScore(true);
            double redScore = SimulatedArena.getInstance().getScore(false);
            double myScore;
            double oppScore;
            //assumes that the robots will be on the opposite side of the player 
            // To-do, find a way around that and get the 
            if(DriverStation.getAlliance().get() == Alliance.Red){
                oppScore = redScore;
                myScore = blueScore;
            }
            else{
                oppScore = blueScore;
                myScore = redScore;
            }
            double[]  currentConditions2 = {oppScore,myScore,timer};
            currentConditions = currentConditions2;
        }
        else{
            double[] currentConditions2 = inputArray.get();
            currentConditions = currentConditions2;
        }
        currentConditions[1]  = currentConditions[1] + biasMyScore;
        currentConditions[2]  = biasTime + currentConditions[0] ;
        currentConditions[0]  = biasOppScore + currentConditions[2] ;
        int nearestNeighbor = 0;
        double nearestNeighborDistance = 10000;
        for(int i = 0; i < data.length; i++){
            double distance = distanceCalculations(currentConditions, data[i]);
            if(distance < nearestNeighborDistance){
                nearestNeighbor = i;
                nearestNeighborDistance = distance;
            }
        }
        return keys[nearestNeighbor];
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

/* 
    public boolean runTests(){
        boolean isGood = true;

        doubele[] inputArray = {0,1,1,2};
        if(calculateNearestNeighbor(inputArray) != null){
            System.out.println("Test 1 failed. Output: ");
            isGood = false;
        }

        double [] inputArray2 = {2.1,1,1,5};
        if(calculateNearestNeighbor(inputArray2) != "End of match park"){
            System.out.println("Test 2 failed. Output: " + calculateNearestNeighbor(inputArray2));
            isGood = false;
        }
        double[] inputArray3 = {1.5,1,0,2};
        if(calculateNearestNeighbor(inputArray3) != "Go play defence"){
            System.out.println("Test 3 failed. Output: " + calculateNearestNeighbor(inputArray3));
            isGood = false;
        }

        double [] inputArray4 = {1.4,0,1,10};
        if(calculateNearestNeighbor(inputArray4) != "Go score"){
            System.out.println("Test 4 failed. Output: " + calculateNearestNeighbor(inputArray4));
            isGood = false;
        }

        return isGood;
    }
        */
}