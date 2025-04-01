package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;




public class ArmSubsystem extends SubsystemBase {
 
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);




 // The P gain for the PID controller that drives this arm.
  private double m_armKp = ArmConstants.kDefaultArmKp;

  private final SparkMax m_motor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_armGearbox);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  

  // Standard classes for controlling our arm


  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          ArmConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
          ArmConstants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          0,
          ArmConstants.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));




  /** Subsystem constructor. */
  public ArmSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
    .inverted(true)
    .encoder
    .positionConversionFactor(1/ArmConstants.kArmReduction)
    .velocityConversionFactor(1);
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ArmConstants.kDefaultArmKp, ArmConstants.kArmKi,ArmConstants.kArmKd);
    /* .maxMotion
        .maxVelocity(2.45)
        .maxAcceleration(2.45)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);
    */

    //.maxMotion?
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);



    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));




  }




  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    //m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            m_armSim.getVelocityRadPerSec())*ArmConstants.kArmReduction,
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }




  /** Load setpoint and kP from preferences. */
  /*
  public void loadPreferences() {//?
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(ArmConstants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(ArmConstants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(ArmConstants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }
  }
  */




  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint(double setPointDegree) {//goal-in degrees?or rad
    /*
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(setPointDegree));
    m_motor.setVoltage(pidOutput);
    */
    m_controller.setReference(Units.degreesToRotations(setPointDegree), ControlType.kPosition);
  }




  public Command setGoal(double degree){
    return run(() -> reachSetpoint(degree));
  }




  public void stop() {
    m_motor.set(0.0);
  }

@Override
public void periodic() {
  SmartDashboard.putNumber("arm rotations", m_encoder.getPosition());
}


public Command setPower(double d) {
  return run(()->m_motor.set(d));
}

public Command tiltDown(double power) {
  return setPower(-power).until(() -> m_encoder.getPosition() <= -0.82)
                         .unless(() -> m_encoder.getPosition() <= -0.82);
}
public Command tiltUp(double power) {
  return setPower(power).until(() -> m_encoder.getPosition() >= -0.01)
                        .unless(() -> m_encoder.getPosition() >= -0.01);
}

//poop

public Command tiltTo(double pos, double pow) {
  return defer(()->{
  if (m_encoder.getPosition() > pos)
    return tiltDown(pow).until(() -> m_encoder.getPosition() <= pos)
    .andThen(setPower(0));
                        // .unless(() -> m_encoder.getPosition() == pos);
  if (m_encoder.getPosition() < pos)
    return tiltUp(pow).until(() -> m_encoder.getPosition() >= pos)
    .andThen(setPower(0));
                        //.unless(() -> m_encoder.getPosition() == pos);
  return setPower(0);
  }).until(()->m_motor.get() == 0);
}



  /*
  public void close() {
    m_motor.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }*/
   
}
