package frc.robot.subsystems.vision.commands.reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightLib;

public class ReefLeft extends Command {
  private final int level;

  double tX;
  double tY;
  double tID;

  public ReefLeft(int level) {
    this.level = level; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tX = LimelightLib.getTX("");
    tY = LimelightLib.getTY("");    
    tID = LimelightLib.getFiducialID("");
    if (tID != 0) {
      //Align straight


      switch (level) {
        case 1:
        //Score L1 Coral logic

        break;

        case 2:
        //Score L2 Coral logic

        break;

        case 3:
        //Score L3 Coral logic

        break;

        case 4: //Assuming we go for L4 Coral
        //Score L4 Coral logic

        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
