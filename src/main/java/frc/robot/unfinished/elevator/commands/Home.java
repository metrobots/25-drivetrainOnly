// package frc.robot.unfinished.elevator.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.unfinished.elevator.Elevator;

// public class Home extends Command {
//     private final Elevator elevator;
//     private boolean isFinished = false;

//     public Home(Elevator elevator) {
//         this.elevator = elevator;
//         addRequirements(elevator);
//     }

//     @Override
//     public void initialize() {
//         isFinished = false;
//         elevator.homeElevator();
//     }

//     @Override
//     public void execute() {
//         // Command will end when bottom limit switch is triggered
//         // handleBottomLimit() in Elevator.java will set isHomed to true
//         if (elevator.isHomed()) {
//             isFinished = true;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         elevator.stopMotors();
//     }

//     @Override
//     public boolean isFinished() {
//         return isFinished;
//     }
// }