package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class AngularPositionHolder {
  private static AngularPositionHolder _instance;

  public static Double _angleToHold = null;
  private PIDController _holdRobotAnglePID = new PIDController(2, 0, 0);
  private Timer _holdRobotAngleTimer = new Timer();

  public static AngularPositionHolder GetInstance(){
    if (_instance == null) {
      _instance = new AngularPositionHolder();
    }

    return _instance;
  }

  public AngularPositionHolder(){
    _holdRobotAngleTimer.start();
   _holdRobotAnglePID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Gets the angular velocity required to return to hold the angle.
   * @return a velocity to correct any dift
   */
  public double getAngularVelocity(double desiredAngularVelocity, double gyroAngleRadians) {
    // robot wants to rotate, so reset everything
    if (desiredAngularVelocity != 0) {
      this.reset();

      return desiredAngularVelocity;
    }

    // wait a quarter second after robots stops being told to rotate
    double correctionPower = 0;
    if (_holdRobotAngleTimer.get() > 0.25) {
      if (_angleToHold == null) {
        _angleToHold = gyroAngleRadians;
      } else {
        correctionPower = _holdRobotAnglePID.calculate(gyroAngleRadians, _angleToHold);
      }
    }

    Logger.recordOutput("Position Holder/AngularPositionHolder AngleToHold", _angleToHold == null ? "null" : "" + _angleToHold);

    return correctionPower;
  }

  public static void setAngleToHoldToCurrentPosition() {
    _angleToHold = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation().getRadians();
  }

  public void reset(){
    _angleToHold = null;
    _holdRobotAngleTimer.reset();
    _holdRobotAnglePID.reset();
  }
}