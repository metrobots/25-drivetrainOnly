// //USE IT LIKE THIS
// //primaryDriver.aButton().onTrue(new SetElevatorPositionCommand(elevator, "L1"));

// package frc.robot.unfinished.elevator.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.unfinished.elevator.Elevator;
// import frc.robot.utils.Constants.ElevatorConstants;

// public class SetPosition extends Command {
//     private final Elevator elevator;
//     private final String level;
//     private double targetPosition;
//     private boolean isHoming = false;
    
//     public SetPosition(Elevator elevator, String level) {
//         this.elevator = elevator;
//         this.level = level.toUpperCase();
//         addRequirements(elevator);
//     }
    
//     @Override
//     public void initialize() {
//         if (!elevator.isHomed()) {
//             isHoming = true;
//             elevator.homeElevator();
//             return;
//         }
        
//         setTargetPosition();
//     }
    
//     private void setTargetPosition() {
//         // Map the level string to the corresponding position
//         switch (level) {
//             case "L1":
//                 targetPosition = ElevatorConstants.L1;
//                 break;
//             case "L2":
//                 targetPosition = ElevatorConstants.L2;
//                 break;
//             case "L3":
//                 targetPosition = ElevatorConstants.L3;
//                 break;
//             case "L4":
//                 targetPosition = ElevatorConstants.L4;
//                 break;
//             default:
//                 System.out.println("Invalid level: " + level);
//                 targetPosition = ElevatorConstants.downPos;
//                 break;
//         }
        
//         elevator.setPositionInches(targetPosition);
//     }
    
//     @Override
//     public void execute() {
//         // If we were homing and just finished, start moving to position
//         if (isHoming && elevator.isHomed()) {
//             isHoming = false;
//             setTargetPosition();
//         }
//     }
    
//     @Override
//     public boolean isFinished() {
//         // If we're homing, wait until homing is complete and we've reached target position
//         if (isHoming) {
//             return false;
//         }
//         return Math.abs(elevator.getHeightInches() - targetPosition) < ElevatorConstants.posTolerance;
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             elevator.stopMotors();
//         }
//     }
// }
