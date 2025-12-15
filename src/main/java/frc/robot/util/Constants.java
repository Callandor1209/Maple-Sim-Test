package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.ParkCommand;

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
          public static final Command[] KEYS = {
            new AutoCommand1(),
            new PlayDefenceCommand(),
            new ParkCommand(Robot.setSim),
            new AutoCommand1(),
            new ParkCommand(Robot.setSim),
            new AutoCommand1(),
            new PlayDefenceCommand(),
            new AutoCommand1(),
            new ParkCommand(Robot.setSim),
            new ParkCommand(Robot.setSim)
          };
    }
}
