// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Shooter;

// public class IndexerCommand extends CommandBase {
//   /** Creates a new IndexerCommand. */

//   private Indexer indexer;
//   private XboxController xbox;
//   //private Shooter Shooter;

//   public IndexerCommand(Indexer subsystem, XboxController xbox) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     indexer = subsystem;
//     //Shooter = subsystem2;
//     addRequirements(indexer);
//     this.xbox = xbox;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() 
//   {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     indexer.autoIndex();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
