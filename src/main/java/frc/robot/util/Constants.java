package frc.robot.util;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.AutoCommand1;
import frc.robot.commands.ParkCommand;
import frc.robot.commands.PlayDefenceCommand;

public final class Constants {
    public static final double CUT_POWER = 0.2;
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
      }


        public final class MLData{
          public static final double[][] DATA = {
            {0,1,1,0},
            {1.5,0,0,1},
            {2.3,1,1,3},
            {1,1,0,3},
            {2.5,0,0,1},
            {2,1,1,6},
            {2,0,1,2},
            {1,0,0,3},
            {4,1,0,10},
            {3.5,0,0,6}
          };
      
          //for each data set need to have an output
          public static final List<Supplier<Command>> KEYS = List.of(
            () ->  new AutoCommand1(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new PlayDefenceCommand( Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new ParkCommand(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new AutoCommand1(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new ParkCommand(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new AutoCommand1(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new PlayDefenceCommand(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new AutoCommand1(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new ParkCommand(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId),
            () -> new ParkCommand(Robot.setDrive,Robot.setVision,Robot.setIntake,Robot.setShooter,Robot.setId)
          );
    }
}
