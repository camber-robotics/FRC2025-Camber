package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    SparkMax intakeMotor = new SparkMax(11, MotorType.kBrushless);

    public IntakeSubsystem()
    {

    }

    public Command setPower(double d)
    {
        return run(()->intakeMotor.set(d));
    }
}
