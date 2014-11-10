
package moe.frc365.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author bradmiller
 */
public class SMDB extends CommandBase {

    public SMDB() {
        // Use requires() here to declare subsystem dependencies
         setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        SmartDashboard.putNumber("FrontLeftVolt", swerveDrive.positionFL.getVoltage());
        SmartDashboard.putNumber("FrontRightVolt", swerveDrive.positionFR.getVoltage());
        SmartDashboard.putNumber("RearLeftVolt", swerveDrive.positionRL.getVoltage());
        SmartDashboard.putNumber("RearRightVolt", swerveDrive.positionRR.getVoltage());
        SmartDashboard.putNumber("FrontLeftturns", swerveDrive.positionFL.getTurns());
        SmartDashboard.putNumber("FrontRightturns", swerveDrive.positionFR.getTurns());
        SmartDashboard.putNumber("RearLeftturns", swerveDrive.positionRL.getTurns());
        SmartDashboard.putNumber("RearRightturns", swerveDrive.positionRR.getTurns());
        SmartDashboard.putNumber("StickX", oi.getJoystickX());
        SmartDashboard.putNumber("StickY", oi.getJoystickY());
        SmartDashboard.putNumber("StickZ", oi.getJoystickZ());
        SmartDashboard.putData("FLpid", swerveDrive.frontLeft);
        SmartDashboard.putNumber("FLError", swerveDrive.frontLeft.getError());
        SmartDashboard.putNumber("FLoutput", swerveDrive.frontLeft.get());
        SmartDashboard.putData("FRpid", swerveDrive.frontRight);
        SmartDashboard.putNumber("FRError", swerveDrive.frontRight.getError());
        SmartDashboard.putNumber("FRoutput", swerveDrive.frontRight.get());
        SmartDashboard.putData("RLpid", swerveDrive.rearLeft);
        SmartDashboard.putNumber("RLError", swerveDrive.rearLeft.getError());
        SmartDashboard.putNumber("RLoutput", swerveDrive.rearLeft.get());
        SmartDashboard.putData("RRpid", swerveDrive.rearRight);
        SmartDashboard.putNumber("RRError", swerveDrive.rearRight.getError());
        SmartDashboard.putNumber("RRoutput", swerveDrive.rearRight.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
