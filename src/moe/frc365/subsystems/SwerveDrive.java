package moe.frc365.subsystems;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import moe.frc365.AnalogChannelVolt;
import moe.frc365.RobotMap;
import moe.frc365.commands.CrabDrive;

/**
 *
 */
public class SwerveDrive extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public AnalogChannelVolt positionFL = new AnalogChannelVolt(1, 4);
    public AnalogChannelVolt positionFR = new AnalogChannelVolt(1, 1);
    public AnalogChannelVolt positionRL = new AnalogChannelVolt(1, 2);
    public AnalogChannelVolt positionRR = new AnalogChannelVolt(1, 3);
    private SpeedController motorSteerFL = new Victor(1, 8);
    private SpeedController motorSteerFR = new Victor(1, 5);
    private SpeedController motorSteerRL = new Victor(1, 6);
    private SpeedController motorSteerRR = new Victor(1, 7);
    public PIDController frontRight = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, positionFR, motorSteerFR, RobotMap.PERIOD);
    public PIDController frontLeft = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, positionFL, motorSteerFL, RobotMap.PERIOD);
    public PIDController rearRight = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, positionRR, motorSteerRR, RobotMap.PERIOD);
    public PIDController rearLeft = new PIDController(RobotMap.P, RobotMap.I,
            RobotMap.D, RobotMap.F, positionRL, motorSteerRL, RobotMap.PERIOD);
    private SpeedController motorDriveFL = new Victor(1, 1);
    private SpeedController motorDriveFR = new Victor(1, 2);
    private SpeedController motorDriveRL = new Victor(1, 4);
    private SpeedController motorDriveRR = new Victor(1, 3);
    //private DigitalModule i2cmodule = new DigitalModule();
    public I2C i2c;
    boolean unwinding = false;
    private double robotAngle = 0.;
    private double X;
    private double Y;
    private double AP;
    private double BP;
    private double CP;
    private double DP;
    private double FL;
    private double FR;
    private double RL;
    private double RR;
    private double FLRatio;
    private double FRRatio;
    private double RLRatio;
    private double RRRatio;
    private boolean driveFront = true;
    private double FLInv = 1.;
    private double FRInv = 1.;
    private double RLInv = 1.;
    private double RRInv = 1.;
    private double FLOffset = 0.;
    private double FROffset = 0.;
    private double RLOffset = 0.;
    private double RROffset = 0.;
    private double W;

    public SwerveDrive() {
        frontRight.setContinuous(RobotMap.CONTINUOUS);
        frontLeft.setContinuous(RobotMap.CONTINUOUS);
        rearRight.setContinuous(RobotMap.CONTINUOUS);
        rearLeft.setContinuous(RobotMap.CONTINUOUS);

        frontRight.setAbsoluteTolerance(RobotMap.TOLERANCE);
        frontLeft.setAbsoluteTolerance(RobotMap.TOLERANCE);
        rearRight.setAbsoluteTolerance(RobotMap.TOLERANCE);
        rearLeft.setAbsoluteTolerance(RobotMap.TOLERANCE);

        frontRight.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        frontLeft.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        rearRight.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);
        rearLeft.setInputRange(RobotMap.POTMIN, RobotMap.POTMAX);

        frontRight.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        frontLeft.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        rearRight.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);
        rearLeft.setOutputRange(-RobotMap.STEERPOW, RobotMap.STEERPOW);

        positionFL.start();
        positionFR.start();
        positionRL.start();
        positionRR.start();

        frontLeft.enable();
        frontRight.enable();
        rearLeft.enable();
        rearRight.enable();

        i2c = DigitalModule.getInstance(1).getI2C(0x04 << 1);
        retrieveOffsets();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new CrabDrive());
    }

    public void toggleFrontBack() {
        driveFront = !driveFront;
        outputLED();
    }

    private void retrieveOffsets() {
        FLOffset = Preferences.getInstance().getDouble("FLOffset", FLOffset);
        FROffset = Preferences.getInstance().getDouble("FROffset", FROffset);
        RLOffset = Preferences.getInstance().getDouble("RLOffset", RLOffset);
        RROffset = Preferences.getInstance().getDouble("RROffset", RROffset);
    }

    public void Crab(double twist, double y, double x, double brake) {
        SmartDashboard.putBoolean("Unwinding", unwinding);
        if (unwinding
                || Math.abs(positionFL.getTurns()) > 5
                || Math.abs(positionFR.getTurns()) > 5
                || Math.abs(positionRL.getTurns()) > 5
                || Math.abs(positionRR.getTurns()) > 5) {
            setDriveSpeed(0., 0., 0., 0.);
            return;
        }

        if (Math.abs(twist) < 1E-6 && Math.abs(y) < 1E-6 && Math.abs(x) < 1E-6) {
            y = 0.05;
        }

        double FWD = y * Math.cos(robotAngle) + x * Math.sin(robotAngle);
        double STR = -y * Math.sin(robotAngle) + x * Math.cos(robotAngle);
        double radius = Math.sqrt(MathUtils.pow(Y, 2) + MathUtils.pow(X, 2));

        AP = STR - twist * X / radius;
        BP = STR + twist * X / radius;
        CP = FWD - twist * Y / radius;
        DP = FWD + twist * Y / radius;

        double FLSetPoint = 2.5;
        double FRSetPoint = 2.5;
        double RLSetPoint = 2.5;
        double RRSetPoint = 2.5;

        if (Math.abs(DP) > 1E-6 || Math.abs(BP) > 1E-6) {
            FLSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(BP, DP));
        }
        if (Math.abs(BP) > 1E-6 || Math.abs(CP) > 1E-6) {
            FRSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(BP, CP));
        }
        if (Math.abs(AP) > 1E-6 || Math.abs(DP) > 1E-6) {
            RLSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(AP, DP));
        }
        if (Math.abs(AP) > 1E-6 || Math.abs(CP) > 1E-6) {
            RRSetPoint = (2.5 + 2.5 / Math.PI * MathUtils.atan2(AP, CP));
        }

        SetSteerSetpoint(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

        FL = Math.sqrt(MathUtils.pow(BP, 2) + MathUtils.pow(DP, 2));
        FR = Math.sqrt(MathUtils.pow(BP, 2) + MathUtils.pow(CP, 2));
        RL = Math.sqrt(MathUtils.pow(AP, 2) + MathUtils.pow(DP, 2));
        RR = Math.sqrt(MathUtils.pow(AP, 2) + MathUtils.pow(CP, 2));

        //Solve for fastest wheel speed
        double speedarray[] = {Math.abs(FL), Math.abs(FR), Math.abs(RL), Math.abs(RR)};

        int arrayLength = 4;
        double maxspeed = speedarray[0];
        for (int i = 1; i < arrayLength; i++) {
            maxspeed = Math.max(maxspeed, speedarray[i]);
        }

        //Set ratios based on maximum wheel speed
        if (maxspeed > 1 || maxspeed < -1) {
            FLRatio = FL / maxspeed;
            FRRatio = FR / maxspeed;
            RLRatio = RL / maxspeed;
            RRRatio = RR / maxspeed;
        } else {
            FLRatio = FL;
            FRRatio = FR;
            RLRatio = RL;
            RRRatio = RR;
        }
        if (brake < -0.5 || y == 0.05 || x == 0.05 || twist == 0.05) {
            FLRatio = 0;
            FRRatio = 0;
            RLRatio = 0;
            RRRatio = 0;
        }


        //Set drive speeds
        setDriveSpeed(FLRatio, -FRRatio, RLRatio, -RRRatio);

    }

    public void Steer(double speed, double angle) {
        // Valid angle input: -Math.PI/2 < angle < Math.PI/2
        // Valid speed input: -1 < speed < 1

        double radius = RobotMap.chassisLength / (2 * Math.cos(Math.PI / 2. - Math.abs(angle))); // Law of cosines

    }

    private void setDriveSpeed(double FLSpeed, double FRSpeed, double RLSpeed, double RRSpeed) {
        if (driveFront) {
            motorDriveFL.set(FLSpeed * FLInv);
            motorDriveFR.set(FRSpeed * FRInv);
            motorDriveRL.set(RLSpeed * RLInv);
            motorDriveRR.set(RRSpeed * RRInv);
        } else {
            motorDriveFL.set(RRSpeed * FLInv);
            motorDriveFR.set(RLSpeed * FRInv);
            motorDriveRL.set(FRSpeed * RLInv);
            motorDriveRR.set(FLSpeed * RRInv);
        }
    }

    private void SetSteerSetpoint(double FLSetPoint, double FRSetPoint, double RLSetPoint, double RRSetPoint) {
        if (driveFront) {
            if (Math.abs(FLSetPoint + FLOffset - positionFL.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + FLOffset - positionFL.getAverageVoltage()) > 3.75) {
                frontLeft.setSetpoint(correctSteerSetpoint(FLSetPoint + FLOffset));
                FLInv = 1;
            } else {
                frontLeft.setSetpoint(correctSteerSetpoint(FLSetPoint + FLOffset - 2.5));
                FLInv = -1;
            }

            if (Math.abs(FRSetPoint + FROffset - positionFR.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + FROffset - positionFR.getAverageVoltage()) > 3.75) {
                frontRight.setSetpoint(correctSteerSetpoint(FRSetPoint + FROffset));
                FRInv = 1;
            } else {
                frontRight.setSetpoint(correctSteerSetpoint(FRSetPoint + FROffset - 2.5));
                FRInv = -1;
            }

            if (Math.abs(RLSetPoint + RLOffset - positionRL.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + RLOffset - positionRL.getAverageVoltage()) > 3.75) {
                rearLeft.setSetpoint(correctSteerSetpoint(RLSetPoint + RLOffset));
                RLInv = 1;
            } else {
                rearLeft.setSetpoint(correctSteerSetpoint(RLSetPoint + RLOffset - 2.5));
                RLInv = -1;
            }

            if (Math.abs(RRSetPoint + RROffset - positionRR.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + RROffset - positionRR.getAverageVoltage()) > 3.75) {
                rearRight.setSetpoint(correctSteerSetpoint(RRSetPoint + RROffset));
                RRInv = 1;
            } else {
                rearRight.setSetpoint(correctSteerSetpoint(RRSetPoint + RROffset - 2.5));
                RRInv = -1;
            }

        } else {

            if (Math.abs(RRSetPoint + FLOffset - positionFL.getAverageVoltage()) < 1.25 || Math.abs(RRSetPoint + FLOffset - positionFL.getAverageVoltage()) > 3.75) {
                frontLeft.setSetpoint(correctSteerSetpoint(RRSetPoint + FLOffset));
                FLInv = 1;
            } else {
                frontLeft.setSetpoint(correctSteerSetpoint(RRSetPoint + FLOffset - 2.5));
                FLInv = -1;
            }

            if (Math.abs(RLSetPoint + FROffset - positionFR.getAverageVoltage()) < 1.25 || Math.abs(RLSetPoint + FROffset - positionFR.getAverageVoltage()) > 3.75) {
                frontRight.setSetpoint(correctSteerSetpoint(RLSetPoint + FROffset));
                FRInv = 1;
            } else {
                frontRight.setSetpoint(correctSteerSetpoint(RLSetPoint + FROffset - 2.5));
                FRInv = -1;
            }

            if (Math.abs(FRSetPoint + RLOffset - positionRL.getAverageVoltage()) < 1.25 || Math.abs(FRSetPoint + RLOffset - positionRL.getAverageVoltage()) > 3.75) {
                rearLeft.setSetpoint(correctSteerSetpoint(FRSetPoint + RLOffset));
                RLInv = 1;
            } else {
                rearLeft.setSetpoint(correctSteerSetpoint(FRSetPoint + RLOffset - 2.5));
                RLInv = -1;
            }

            if (Math.abs(FLSetPoint + RROffset - positionRR.getAverageVoltage()) < 1.25 || Math.abs(FLSetPoint + RROffset - positionRR.getAverageVoltage()) > 3.75) {
                rearRight.setSetpoint(correctSteerSetpoint(FLSetPoint + RROffset));
                RRInv = 1;
            } else {
                rearRight.setSetpoint(correctSteerSetpoint(FLSetPoint + RROffset - 2.5));
                RRInv = -1;
            }
        }
    }

    private double correctSteerSetpoint(double setpoint) {
        if (setpoint < 0) {
            return (setpoint + 5);
        } else if (setpoint > 5) {
            return (setpoint - 5);
        } else if (setpoint == 5) {
            return 0;
        } else {
            return setpoint;
        }
    }

    public void outputLED() {
        i2c.write(0x0, 40 * (driveFront ? 1 : 0));
    }

    public void setWheelbase(double w, double x, double y) {
        W = w;
        X = x;
        Y = y;
    }

    public boolean unwind() {
        boolean retval = false;
        unwinding = true;
        retval = unwindWheel(positionFL, frontLeft) || unwindWheel(positionFR, frontRight) || unwindWheel(positionRL, rearLeft) || unwindWheel(positionRR, rearRight);
        if (!retval) {
            unwinding = false;
        }
        return retval;
    }

    private boolean unwindWheel(AnalogChannelVolt wheel, PIDController pid) {
        double temp;
        double turns = wheel.getTurns();
        if (turns > 1) {
            temp = wheel.getAverageVoltage() - 1.0;
            if (temp < 0.0) {
                temp = 5.0 + temp;
            }
            pid.setSetpoint(temp);
            return true;
        } else if (turns < -1) {
            temp = wheel.getAverageVoltage() + 1.0;
            if (temp > 5.0) {
                temp = temp - 5.0;
            }
            pid.setSetpoint(temp);
            return true;
        } else {
            return false;
        }
    }

    public void doneUnwind() {
        unwinding = false;
    }

    public void angleUp() {
        robotAngle += 0.1;
        if (robotAngle > 360.) {
            robotAngle = 0.;
        }
        outputLED();
    }

    public void angleDown() {
        robotAngle -= 0.1;
        if (robotAngle > 360.) {
            robotAngle = 0.;
        }
        outputLED();
    }

    public boolean resetTurns() {
        positionFR.resetTurns();
        positionFL.resetTurns();
        positionRR.resetTurns();
        positionRL.resetTurns();
        robotAngle = 0.;
        return true;

    }

    public void setOffsets(double FLOff, double FROff, double RLOff, double RROff) {
        FLOffset = FLOff;
        FROffset = FROff;
        RLOffset = RLOff;
        RROffset = RROff;
        Preferences.getInstance().putDouble("FLOffset", FLOffset);
        Preferences.getInstance().putDouble("FROffset", FROffset);
        Preferences.getInstance().putDouble("RLOffset", RLOffset);
        Preferences.getInstance().putDouble("RROffset", RROffset);
        Preferences.getInstance().save();
    }
}
