package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;


// public class Winch {
//     private final SparkMax winchMotor;

//     public Winch() {
//         winchMotor = new SparkMax(TunerConstants.getHangingMotorID(), SparkMax.MotorType.kBrushless);
//     }

//     private void turnIn() {
//         winchMotor.set(-0.3);
//     }

//     private void turnOut() {
//         winchMotor.set(0.3);
//     }

//     private void stopMotor() {
//         winchMotor.set(0);
//     }

//     public InstantCommand turnInCommand() {
//         return new InstantCommand(() -> turnIn());
//     }

//     public InstantCommand turnOutCommand() {
//         return new InstantCommand(() -> turnOut());
//     }

//     public InstantCommand stopMotorCommand() {
//         return new InstantCommand(() -> stopMotor());
//     }
// }
