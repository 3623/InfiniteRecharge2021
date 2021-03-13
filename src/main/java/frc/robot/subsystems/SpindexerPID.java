package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

// TODO Update system to be a drop in replacement for Spindexer to run on PID

public class SpindexerPID extends PIDSubsystem {
    WPI_VictorSPX motor;
    public static final double SHOOT_TIME = 1.5;
    public static final double INDEX_TIME = 5.0;
    private static final double INDEX_SPEED = 0.4;
    private static final double SHOOT_SPEED = 0.9;
    private boolean indexing = false;
    private boolean shooting = false;
    private boolean readying = false;
    private Encoder encoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0 / TICKS_PER_ENCODER_REV // Converts to Output Revolution Count
                                                        * (1.0 / (400/24)) // Converts to Whole Spindexer Revolution Count
                                                        / 5; // Converts to Spindexer Sections

    private static final double kP = 1.3 / 20.0;
    private static final double kI = kP / 500.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = .25;
    private int roundedPosition;
    private double tenthsOfRounded;

    public SpindexerPID() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        motor = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        encoder = new Encoder(4, 5);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void monitor() {
        SmartDashboard.putNumber("Current Segment", encoder.getDistance() % 5 + 1);
        updatePositionVars();
    }

    private void updatePositionVars(){
        roundedPosition = getWholeSegments();
        tenthsOfRounded = getTenthsOfSegments();
    }

    private int getWholeSegments(){
        return (int) Math.round(encoder.getDistance());
    }

    private double getTenthsOfSegments(){
        return encoder.getDistance() - roundedPosition;
    }

    /**
     * @param indexing True if trying to shoot
     */
    public void setIndexing(boolean indexing) {
        this.indexing = indexing;
        chooseOutput();
    }

    /**
     * @param shooting True if trying to shoot
     */
    public void setShooting(boolean shooting) {
        this.shooting = shooting;
        chooseOutput();
    }

    /**
     * @param readyUp True if trying to be in position to shoot
     */
    public void setReadying(boolean readying) {
        this.readying = readying;
        chooseOutput();
    }

    public boolean isReady(){
        boolean readyState = false;
        if (tenthsOfRounded > 0.9 || tenthsOfRounded < 0.1)
            readyState = true;
        return readyState;
    }

    private void chooseOutput() {
        if (shooting) setSpinning(SHOOT_SPEED);
        else if (indexing) setSpinning(INDEX_SPEED);
        else if (isReady() && readying) setSpinningPID(encoder.getDistance());
        else if (!isReady() && readying) setSpinningPID(roundedPosition+1);
        else setSpinningPID(encoder.getDistance());
    }

    private void setSpinning(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    private void setSpinningPID(double position){
        motor.set(ControlMode.Position, position);
    }

    public boolean getShooting(){
        return shooting;
    }

    public void stop(){
        motor.set(ControlMode.PercentOutput, 0.0);
        shooting = false;
        indexing = false;
        readying = false;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub

    }

    @Override
    protected double getMeasurement() {
        return encoder.getDistance();
    }
}
