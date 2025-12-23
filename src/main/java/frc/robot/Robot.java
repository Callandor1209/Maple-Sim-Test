// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.AIRobot2;
import frc.robot.subsystems.AIRobotSimulation;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.MLClass;


public class Robot extends LoggedRobot {
  public static int blueScore = 0;
  private Command m_autonomousCommand;
  public static Timer matchTimer = new Timer();
  public static boolean noDefault = false;
  public static List<AIRobot2> aiRobot2List = new ArrayList<>();

 public static final  PPHolonomicDriveController holoConfig = new PPHolonomicDriveController(
    new PIDConstants(5.0, 0.0, 0.0),  
    new PIDConstants(5.0, 0.0, 0.0)  
);

public static MapleSimSwerve DRIVETRAIN_SUBSYSTEM;
public static IntakeSubsystem INTAKE_SUBSYSTEM;
public static ShooterSubsystem SHOOTER_SUBSYSTEM;
public static VisionSubsystem VISION_SUBSYSTEM;
public static MLClass ML_CLASS;
public static MapleSimSwerve setDrive;
public static ShooterSubsystem setShooter;
public static VisionSubsystem setVision;
public static IntakeSubsystem setIntake;
public static int setId;
  public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
      public static final SwerveRequest.ApplyRobotSpeeds PATH_APPLY_ROBOT_SPEEDS = new SwerveRequest.ApplyRobotSpeeds();
      public static final SwerveRequest.FieldCentric SWERVE_REQUEST_DRIVE = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

      public Pose3d pose3d;


  public Robot() {
Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
matchTimer.start();

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
} else {

    Logger.addDataReceiver(new NT4Publisher());
}

Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  SmartDashboard.putData("Auto choices", m_chooser);
  if(Robot.isSimulation()){
    SimulatedArena.overrideInstance(new Arena2024Crescendo());
    SimulatedArena.getInstance();
  }
  DRIVETRAIN_SUBSYSTEM = new MapleSimSwerve(true, new Pose2d(2,2,new Rotation2d()));
  INTAKE_SUBSYSTEM = new IntakeSubsystem(DRIVETRAIN_SUBSYSTEM.returnSwerveThing());
  SHOOTER_SUBSYSTEM = new ShooterSubsystem(Robot.INTAKE_SUBSYSTEM,Robot.DRIVETRAIN_SUBSYSTEM.returnSwerveThing());
  configureBindings();
  //DRIVETRAIN_SUBSYSTEM.setDefaultCommand(new DriveTrainDefaultCommand(DRIVETRAIN_SUBSYSTEM));
  VISION_SUBSYSTEM = new VisionSubsystem(DRIVETRAIN_SUBSYSTEM.returnSwerveThing(),"1");
  ML_CLASS = new MLClass();
  ML_CLASS.train();


  
  

  }

  @Override
  public void robotPeriodic() {
     CommandScheduler.getInstance().run();
            Pose2d pose2d = DRIVETRAIN_SUBSYSTEM.getPose();
             pose3d = new Pose3d(
        pose2d.getX(), 
        pose2d.getY(), 
        0.0,
        new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
        Logger.recordOutput("Robot/Pose3d", pose3d);




        Pose3d[] notesPoses = SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Note");
  Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

    aiRobot2List.add(new AIRobot2(0, "0", new Pose2d(3,3, new Rotation2d())));
    aiRobot2List.add(new AIRobot2(1, "1",new Pose2d(5,5, new Rotation2d())));
    aiRobot2List.add(new AIRobot2(2, "2", new Pose2d(7,7, new Rotation2d())));

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  SimulatedArena.getInstance().simulationPeriodic();
  RoboRioSim.setVInCurrent(0);
  VISION_SUBSYSTEM.simUpdate();
  }

  public void configureBindings(){
    m_driverController.cross().onTrue(new Command() {
      public void initialize() {
        double doubleX = Math.random() * 20;
        if(doubleX > 16){
          doubleX = 3;
        }
        double doubleY = Math.random() * 10;
        if(doubleY > 8){
          doubleY = 5;
        }
        SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(doubleX, doubleY)));
        System.out.println("Adding Piece");
      }
      public boolean isFinished(){
        return true;
      }
    });
    m_driverController.R1().whileTrue(new IntakeCommand());
    m_driverController.L1().onTrue(new InstantCommand(){
      @Override
      public void initialize() {
        SHOOTER_SUBSYSTEM.launchNote();
      }
    });
  }
}
