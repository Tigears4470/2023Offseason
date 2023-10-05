package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.K_ExtSub;
import frc.robot.Constants.K_PivotSub;

public class PivotSubPID extends SubsystemBase{
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
  //           YES IT LOOKS WRONG BUT BLAME ELECTRICAL FOR THEIR WIRING!
  //           --> DEFAULT IS ALWAYS TRUE BUT WHEN HIT THEY RETURN FALSE!
  private final DigitalInput BtmLimit = new DigitalInput(0);
  private final DigitalInput TopLimit = new DigitalInput(1);

  // Limits range of motion
  private double desiredAngle = 0;
  private double maxAngle = 95;
  private double minAngle = 2.5;
  
  // === Shuffleboard ===
  // Dev Tab
  private final ShuffleboardLayout devTab = Shuffleboard.getTab("DevTab").getLayout("Pivot", BuiltInLayouts.kList);
  //    display PID coefficients on SmartDashboard
  private GenericEntry entryPivotPGain;
  private GenericEntry entryPivotIGain;
  private GenericEntry entryPivotDGain;
  private GenericEntry entryPivotIZone;
  private GenericEntry entryPivotFeedForward;
  private GenericEntry entryPivotMaxOutput;
  private GenericEntry entryPivotMinOutput;
  //    display Smart Motion coefficients
  private GenericEntry entryPivotMaxVelocity;
  private GenericEntry entryPivotMinVelocity;
  private GenericEntry entryPivotMaxAcceleration;
  private GenericEntry entryPivotAllowedClosedLoopError;

  // Driver's Tab
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Driver's Tab");
  private GenericEntry entryPivotBottomLimit;
  private GenericEntry entryPivotTopLimit;
  private GenericEntry entryPivotEncoder;
  private GenericEntry entryPivotDesiredAngle;


  public PivotSubPID(){
    if(K_PivotSub.isUsingPivot){
      motor = new CANSparkMax(5, MotorType.kBrushless);
      encoder = motor.getEncoder();
      pid = motor.getPIDController();
      motor.setIdleMode(IdleMode.kBrake);

      // set conversion factor so getPosition returns degrees
      encoder.setPositionConversionFactor(360.0/K_PivotSub.gearRatio);

      encoder.setPosition(0);
      desiredAngle = encoder.getPosition();

      // PID coefficients
      kP = 0.0001515; 
      kI = 0.0000005;
      kD = 0; 
      kIz = 0.005; 
      kFF = 0.0003; 
      kMaxOutput = 2; 
      kMinOutput = -2;
      // Smart Motion Coefficients
      double rps = 0.2;
      maxVel = rps*60*60; // rpm: .3rps -> 12 rpm -> (adjusted by gear ratio)
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

      initShuffleboard();
    }
  }

  //Return the encoder
  public RelativeEncoder getEncoder(){
    return encoder;
  }

  //Return the maxAngle
  public double getMaxAngle(){
    return maxAngle;
  }

  // sets the desired angle to set angle to
  // 0 - 100 degrees
  public void setAngle (double angle) {
    if(K_PivotSub.isUsingPivot){
      if (angle < minAngle)
        angle = minAngle;
      if (angle > maxAngle)
        angle = maxAngle;
      desiredAngle = angle;
    }
    pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
  }

  //Returns the current angle of the pivot
  public double getCurrentAngle(){
    if(K_PivotSub.isUsingPivot)
      return encoder.getPosition();
    return 0.0;
  }

  //Returns the current desired angle
  public double getDesiredAngle(){
    if(K_PivotSub.isUsingPivot)
      return desiredAngle;
    return 0.0;
  }

  //Returns true or false depending on whether the arm's current position is within a tolerance of its desired position
  public boolean withinTolerance() {
    return Math.abs((getDesiredAngle()-getCurrentAngle())) < K_PivotSub.tolerance;
  }

  // Changes angle to aim for
  // If change is past min or max in either direction revert the change
  public void changeAngle (double increment) {
    if(K_PivotSub.isUsingPivot){
      if ((increment > 0 && TopLimit.get()) || (increment < 0 && BtmLimit.get())) {
        desiredAngle += increment;
      } else if (!TopLimit.get()) {
        maxAngle = encoder.getPosition();
      } else if (!BtmLimit.get()) {
        minAngle = encoder.getPosition();
      }
      if (desiredAngle > maxAngle) 
        desiredAngle= maxAngle;
      if (desiredAngle < minAngle) 
        desiredAngle= minAngle;
      pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
    }
  }


  // Stops the motor in case of emergency
  public void emergencyStop() {
    if(K_ExtSub.isUsingExt){
      motor.stopMotor();
    }
  }

  public void initShuffleboard() {
    // Driver's Tab
    entryPivotBottomLimit = mainTab.add("Pivot Bottom Limit", BtmLimit.get()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    entryPivotTopLimit = mainTab.add("Pivot Top Limit", TopLimit.get()).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    entryPivotEncoder = mainTab.add("Pivot Encoder", encoder.getPosition()).withWidget(BuiltInWidgets.kTextView).getEntry();
    entryPivotDesiredAngle = mainTab.add("Pivot Desired Angle", desiredAngle).withWidget(BuiltInWidgets.kTextView).getEntry();

    // Pivot Dev Tab
    if (K_PivotSub.devMode) {
      // display PID coefficients on SmartDashboard
      entryPivotPGain = devTab.add("Pivot P Gain", kP).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotIGain = devTab.add("Pivot I Gain", kI).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotDGain = devTab.add("Pivot D Gain", kD).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotIZone = devTab.add("Pivot I Zone", kIz).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotFeedForward = devTab.add("Pivot Feed Forward", kFF).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotMaxOutput = devTab.add("Pivot Max Output", kMaxOutput).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotMinOutput = devTab.add("Pivot Min Output", kMinOutput).withWidget(BuiltInWidgets.kTextView).getEntry();

      // display Smart Motion coefficients        // display Smart Motion coefficients
      entryPivotMaxVelocity = devTab.add("Pivot Max Velocity", maxVel).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotMinVelocity = devTab.add("Pivot Min Velocity", minVel).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotMaxAcceleration = devTab.add("Pivot Max Acceleration", maxAcc).withWidget(BuiltInWidgets.kTextView).getEntry();
      entryPivotAllowedClosedLoopError = devTab.add("Pivot Allowed Closed Loop Error", allowedErr).withWidget(BuiltInWidgets.kTextView).getEntry();
      devTab.add("Pivot Set Position", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

      devTab.add("Mode", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    }
  }

  public void updateShuffleboard() {
    // Driver's Tab
    entryPivotBottomLimit.setBoolean(BtmLimit.get());
    entryPivotTopLimit.setBoolean(TopLimit.get());
    entryPivotEncoder.setDouble(encoder.getPosition());
    entryPivotDesiredAngle.setDouble(desiredAngle);

    // Pivot Dev Tab 
    if (K_PivotSub.devMode) {
      double p = entryPivotPGain.getDouble(0);
      double i = entryPivotIGain.getDouble(0);
      double d = entryPivotDGain.getDouble(0);
      double iz = entryPivotIZone.getDouble(0);
      double ff = entryPivotFeedForward.getDouble(0);
      double max = entryPivotMaxOutput.getDouble(0);
      double min = entryPivotMinOutput.getDouble(0);
      double maxV = entryPivotMaxVelocity.getDouble(0);
      double minV = entryPivotMinVelocity.getDouble(0);
      double maxA = entryPivotMaxAcceleration.getDouble(0);
      double allE = entryPivotAllowedClosedLoopError.getDouble(0);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pid.setP(p); kP = p; }
      if((i != kI)) { pid.setI(i); kI = i; }
      if((d != kD)) { pid.setD(d); kD = d; }
      if((iz != kIz)) { pid.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { pid.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        pid.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
      if((maxV != maxVel)) { pid.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
      if((minV != minVel)) { pid.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
      if((maxA != maxAcc)) { pid.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
      if((allE != allowedErr)) { pid.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    }
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    pid.setReference(desiredAngle, CANSparkMax.ControlType.kSmartMotion);
  }
}
