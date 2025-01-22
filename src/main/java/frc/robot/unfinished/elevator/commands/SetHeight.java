// package frc.robot.unfinished.elevator.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.unfinished.elevator.Elevator;


// public class SetHeight extends Command {
//     private final Elevator elevator;
//     private final double targetHeightInches;

//     public SetHeight(Elevator elevator, double heightInches) {
//         this.elevator = elevator;
//         this.targetHeightInches = heightInches;
//         addRequirements(elevator);
//     }

//     @Override
//     public void initialize() {
//         elevator.setPositionInches(targetHeightInches);
//     }

//     @Override
//     public boolean isFinished() {
//         return elevator.isAtHeight(targetHeightInches);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             elevator.stopMotors();
//         }
//     }
// }