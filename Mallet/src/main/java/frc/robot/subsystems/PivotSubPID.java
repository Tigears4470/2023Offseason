package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.K_ExtSub;
import frc.robot.Constants.K_PivotSub;

public class PivotSubPID extends SubsystemBase {
  // These are the Pivot Motors
  // Idle - Break on both
  // ID's 5 & 6
  private final CANSparkMax motor;
  private final SparkMaxPIDController pid;
  private final RelativeEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
  // This is the motorControllerGroup of the 2 prior motors
  // Intended to make the Pivot Point Turn

  // Limit Switches
  // WARNING - MAKE SURE THE LIMITS ARE HAVING THE YELLOW IN GROUND!
  // YES IT LOOKS WRONG BUT BLAME ELECTRICAL FOR THEIR WIRING!
  // --> DEFAULT IS ALWAYS TRUE BUT WHEN HIT THEY RETURN FALSE!
  private final DigitalInput BtmLimit = new DigitalInput(0);
  private final DigitalInput TopLimit = new DigitalInput(1);

  // Limits range of motion
  private double desiredAngle = 0;
  private double maxAngle = 95;
  private double minAngle = 2.5;

  public PivotSubPID() {
    if (K_PivotSub.isUsingPivot) {
      motor = new CANSparkMax(5, MotorType.kBrushless);
      encoder = motor.getEncoder();
      pid = motor.getPIDController();
      motor.setIdleMode(IdleMode.kBrake);

      // set conversion factor so getPosition returns degrees
      encoder.setPositionConversionFactor(360.0 / K_PivotSub.gearRatio);
      // set conversion ratio to 1 ONLY FOR CALIBRATING FOR ANGLE
      // encoder1.setPositionConversionFactor(1);

      encoder.setPosition(0);
      desiredAngle = encoder.getPosition();

      // PID coefficients
      kP = 0.0002015;
      kI = 0.0000005;
      kD = 0;
      kIz = 0.005;
      kFF = 0.00035;
      kMaxOutput = 2;
      kMinOutput = -2;
      // Smart Motion Coefficients
      double rps = 0.2;
      maxVel = rps * 60 * 60; // rpm: .3rps -> 12 rpm -> (adjusted by gear ratio)
      maxAcc = 1440;

      // set PID coefficients
      pid.setP(kP);
      pid.setI(kI);
      pid.setD(kD);
      pid.setIZone(kIz);
      pid.setFF(kFF);
      pid.setOutputRange(kMinOutput, kMaxOutput);

      /**
       * Smart Motion coefficients are set on a SparkMaxPIDController object
       * 
       * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
       * the pid controller in Smart Motion mode
       * - setSmartMotionMinOutputVelocity() will put a lower bound in
       * RPM of the pid controller in Smart Motion mode
       * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
       * of the pid controller in Smart Motion mode
       * - setSmartMotionAllowedClosedLoopError() will set the max allowed
       * error for the pid controller in Smart Motion mode
       */
      int smartMotionSlot = 0;
      pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

      if (K_PivotSub.devMode) {
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Pivot P Gain", kP);
        SmartDashboard.putNumber("Pivot I Gain", kI);
        SmartDashboard.putNumber("Pivot D Gain", kD);
        SmartDashboard.putNumber("Pivot I Zone", kIz);
        SmartDashboard.putNumber("Pivot Feed Forward", kFF);
        SmartDashboard.putNumber("Pivot Max Output", kMaxOutput);
        SmartDashboard.putNumber("Pivot Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Pivot Max Velocity", maxVel);
        SmartDashboard.putNumber("Pivot Min Velocity", minVel);
        SmartDashboard.putNumber("Pivot Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Pivot Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Pivot Set Position", 0);
      }

      // button to toggle between velocity and smart motion modes

      SmartDashboard.putBoolean("Mode", true);

    }
  }

  // Return the encoder
  public RelativeEncoder getEncoder() {
    return encoder;
  }

  // Return the maxAngle
  public double getMaxAngle() {
    return maxAngle;
  }

  // sets the desired angle to set angle to
  // 0 - 100 degrees
  public void setAngle(double angle) {
    if (K_PivotSub.isUsingPivot) {
      if (angle < minAngle)
        angle = minAngle;
      if (angle > maxAngle)
        angle = maxAngle;
      desiredAngle = angle;
    }
    pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
  }

  // Returns the current angle of the pivot
  public double getCurrentAngle() {
    if (K_PivotSub.isUsingPivot)
      return encoder.getPosition();
    return 0.0;
  }

  // Returns the current desired angle
  public double getDesiredAngle() {
    if (K_PivotSub.isUsingPivot)
      return desiredAngle;
    return 0.0;
  }

  // Returns true or false depending on whether the arm's current position is
  // within a tolerance of its desired position
  public boolean withinTolerance() {
    return Math.abs((getDesiredAngle() - getCurrentAngle())) < K_PivotSub.tolerance;
  }

  // Changes angle to aim for
  // If change is past min or max in either direction revert the change
  public void changeAngle(double increment) {
    if (K_PivotSub.isUsingPivot) {
      if ((increment > 0 && TopLimit.get()) || (increment < 0 && BtmLimit.get())) {
        desiredAngle += increment;
      } else if (!TopLimit.get()) {
        maxAngle = encoder.getPosition();
      } else if (!BtmLimit.get()) {
        minAngle = encoder.getPosition();
      }
      if (desiredAngle > maxAngle)
        desiredAngle = maxAngle;
      if (desiredAngle < minAngle)
        desiredAngle = minAngle;
      pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  // Stops the motor in case of emergency
  public void emergencyStop() {
    if (K_ExtSub.isUsingExt) {
      motor.stopMotor();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Bottom Limit", BtmLimit.get());
    SmartDashboard.putNumber("Pivot Pivot Encoder", encoder.getPosition());
    SmartDashboard.putBoolean("Top Limit", TopLimit.get());
    SmartDashboard.putNumber("Pivot Desired Angle", desiredAngle);
    if (K_PivotSub.devMode) {
      double p = SmartDashboard.getNumber("Pivot P Gain", 0);
      double i = SmartDashboard.getNumber("Pivot I Gain", 0);
      double d = SmartDashboard.getNumber("Pivot D Gain", 0);
      double iz = SmartDashboard.getNumber("Pivot I Zone", 0);
      double ff = SmartDashboard.getNumber("Pivot Feed Forward", 0);
      double max = SmartDashboard.getNumber("Pivot Max Output", 0);
      double min = SmartDashboard.getNumber("Pivot Min Output", 0);
      double maxV = SmartDashboard.getNumber("Pivot Max Velocity", 0);
      double minV = SmartDashboard.getNumber("Pivot Min Velocity", 0);
      double maxA = SmartDashboard.getNumber("Pivot Max Acceleration", 0);
      double allE = SmartDashboard.getNumber("Pivot Allowed Closed Loop Error", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to
      // controller
      if ((p != kP)) {
        pid.setP(p);
        kP = p;
      }
      if ((i != kI)) {
        pid.setI(i);
        kI = i;
      }
      if ((d != kD)) {
        pid.setD(d);
        kD = d;
      }
      if ((iz != kIz)) {
        pid.setIZone(iz);
        kIz = iz;
      }
      if ((ff != kFF)) {
        pid.setFF(ff);
        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        pid.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
      }
      if ((maxV != maxVel)) {
        pid.setSmartMotionMaxVelocity(maxV, 0);
        maxVel = maxV;
      }
      if ((minV != minVel)) {
        pid.setSmartMotionMinOutputVelocity(minV, 0);
        minVel = minV;
      }
      if ((maxA != maxAcc)) {
        pid.setSmartMotionMaxAccel(maxA, 0);
        maxAcc = maxA;
      }
      if ((allE != allowedErr)) {
        pid.setSmartMotionAllowedClosedLoopError(allE, 0);
        allowedErr = allE;
      }
      // desiredAngle = SmartDashboard.getNumber("Pivot Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
    }

  }
}
