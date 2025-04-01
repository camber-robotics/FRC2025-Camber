// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final ElevatorSubsystem     elevator           = new ElevatorSubsystem();
  private final ArmSubsystem          arm                = new ArmSubsystem();
  private final IntakeSubsystem       intake             = new IntakeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    //elevator.setDefaultCommand(elevator.setGoal(0.5));
    arm.setDefaultCommand(arm.setGoal(0));
    configureBindings();
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
    NamedCommands.registerCommand("test", Commands.print("Hello World"));
  }

// The real world (whats that?)

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() *-1 ,

                                                                () -> m_driverController.getLeftX() * -1) // set to 0 
                                                                .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) // m_driverController::getRightX
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(.8)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
//Non reality code


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

        m_driverController.button(17).whileTrue( //True lazy button
          drivebase.driveToPose(//Coral side 1
          new Pose2d(new Translation2d(Meter.of(3.4),Meter.of(5.1)), Rotation2d.fromDegrees(-50)))
        .andThen(
          drivebase.driveToPose(//Human Player station
          new Pose2d(new Translation2d(Meter.of(1),Meter.of(7)), Rotation2d.fromDegrees(130)))
        ).andThen(
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(4.3), Meter.of(6.3)), Rotation2d.fromDegrees(-50)))
        ).andThen(
          drivebase.driveToPose( //Coral side 2
          new Pose2d(new Translation2d(Meter.of(5.2), Meter.of(5.2)), Rotation2d.fromDegrees(-120)))
         ).andThen(
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(4.3), Meter.of(6.3)), Rotation2d.fromDegrees(130)))
        ).andThen(
          drivebase.driveToPose(//Human Player station
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(7)), Rotation2d.fromDegrees(130)))
        ).andThen (
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(6.5), Meter.of(5.6)), Rotation2d.fromDegrees(130)))
        ).andThen(//Coral side 3
          drivebase.driveToPose(new Pose2d(new Translation2d(Meter.of(6.1), Meter.of(4)), Rotation2d.fromDegrees(180)))
        )

        );

        //elevator
        elevator.setDefaultCommand(elevator.setPower(0));
        m_operatorController.povDown().whileTrue(elevator.goDown(0.7));
        m_operatorController.povUp().whileTrue(elevator.goUp(0.7));

        //intake/outtake
        intake.setDefaultCommand(intake.setPower(0));
        m_operatorController.leftBumper().whileTrue(intake.setPower(0.4));
        m_operatorController.rightBumper().whileTrue(intake.setPower(-0.4));
        m_operatorController.x().whileTrue(intake.setPower(-0.2)); //slow outtake
        //m_operatorController.b().whileTrue(arm.tiltTo(-0.77485,0.5));
        
        //arm up/down
        arm.setDefaultCommand(arm.setPower(0));
        m_operatorController.a().whileTrue(arm.tiltDown(.3));
        m_operatorController.y().whileTrue(arm.tiltUp(.3));
        
        //L2- left d-pad   added 3/18/2025
        m_operatorController.povLeft().onTrue(arm.tiltTo(-0.76, .5));
        m_operatorController.povLeft().onTrue(elevator.goTo(0, 0.9));
         
        //L3 - right d-pad
        m_operatorController.povRight().onTrue(arm.tiltTo(-0.788, .5));
        m_operatorController.povRight().onTrue(elevator.goTo(11.433, 0.8));

        //intake? - b
        m_operatorController.b().onTrue(elevator.goTo(0,0.8));
        m_operatorController.b().onTrue(arm.tiltTo(-0.69, .5)); 

        //reset - right trigger
        m_operatorController.rightTrigger().onTrue(elevator.goTo(0, 0.6));
        m_operatorController.rightTrigger().onTrue(arm.tiltTo(-0.05,0.8));
       
        //auto lines
        NamedCommands.registerCommand("raiseElevator", elevator.goTo(0.3,0.3));//.withTimeout(0.2));
        NamedCommands.registerCommand("out", intake.setPower(-0.3).repeatedly());//.withTimeout(1));//.withTimeout(1));
        NamedCommands.registerCommand("armL2", arm.tiltTo(-0.76, 0.6));//.withTimeout(0.2));
        NamedCommands.registerCommand("outPrint", Commands.print("IAMOUT"));//.withTimeout(1));
        NamedCommands.registerCommand("armIntake", arm.tiltTo(-0.69, 0.4));//.withTimeout(0.2));

  }
        
   /* Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("right shoot test"); //new new auto worked in comp; right shoot test
  }

   public ParallelCommandGroup setElevArm (double goal, double degree){
  return  new ParallelCommandGroup(elevator.setGoal(goal), arm.setGoal(degree));
 }


}
