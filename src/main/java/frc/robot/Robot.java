// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveTrainDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

public static SwerveDrive DRIVETRAIN_SUBSYSTEM;
public static IntakeSubsystem INTAKE_SUBSYSTEM;
public static ShooterSubsystem SHOOTER_SUBSYSTEM;
  public static final CommandPS5Controller m_driverController = new CommandPS5Controller(0);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
      public static final SwerveRequest.ApplyRobotSpeeds PATH_APPLY_ROBOT_SPEEDS = new SwerveRequest.ApplyRobotSpeeds();
      public static final SwerveRequest.FieldCentric SWERVE_REQUEST_DRIVE = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

      public Pose3d pose3d;
// Create and configure a drivetrain simulation configuration
final static DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(COTS.ofMark4(
                DCMotor.getKrakenX60(2), // Drive motor is a Kraken X60
                DCMotor.getKrakenX60(2), // Steer motor is a Falcon 500
                COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));

        /* Create a swerve drive simulation */
public static SwerveDriveSimulation swerveDriveSimulation = new SwerveDriveSimulation(
  // Specify Configuration
  driveTrainSimulationConfig,
  // Specify starting pose
  new Pose2d(3, 3, new Rotation2d())
);


  public Robot() {
Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value


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
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
  }
  DRIVETRAIN_SUBSYSTEM = new MapleSimSwerve();
  INTAKE_SUBSYSTEM = new IntakeSubsystem(swerveDriveSimulation);
  SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  configureBindings();
  DRIVETRAIN_SUBSYSTEM.setDefaultCommand(new DriveTrainDefaultCommand(DRIVETRAIN_SUBSYSTEM));

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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  SimulatedArena.getInstance().simulationPeriodic();
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
