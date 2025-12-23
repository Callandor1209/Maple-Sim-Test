// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.List;


import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ParkCommand extends Command {
  /** Creates a new ParkCommand. */
  MapleSimSwerve driveTrainSubsystem;
  VisionSubsystem visionSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  ChassisSpeeds speeds;
  Command followPath;
  int id;

  public ParkCommand(MapleSimSwerve driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveSimulation;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.id = id;
    addRequirements(driveSimulation,visionSubsystem,intakeSubsystem,shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( 
      new Pose2d(driveTrainSubsystem.getPose().getX(),driveTrainSubsystem.getPose().getY(), new Rotation2d()),
      new Pose2d(2,7,new Rotation2d())
    );
    PathConstraints pathConstraints = new PathConstraints(0.7, 1, 540, 720); //this is just pulled from pathplanner defaults for now
    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0,new Rotation2d()));
    try {
      followPath =  new FollowPathCommand(
        path,
        () -> driveTrainSubsystem.getPose(),
        () -> driveTrainSubsystem.getMeasuredSpeeds(),
        (speeds,feedforwards) -> driveTrainSubsystem.drive(speeds, false, false),
        Robot.holoConfig, 
        RobotConfig.fromGUISettings(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
 );
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    followPath.schedule();
    }





  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("In Park Command");
    speeds = driveTrainSubsystem.getMeasuredSpeeds();
    if(followPath != null){
      followPath.execute();
      if(followPath.isFinished()){
        followPath.end(false);
        followPath = null;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiRobot2List.get(id).enabled = false;;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speeds.vxMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0 && speeds.vyMetersPerSecond == 0;
  }
}
