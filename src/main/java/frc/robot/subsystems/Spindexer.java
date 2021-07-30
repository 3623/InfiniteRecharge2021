/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.util.Utils;
import frc.util.Utils.MovingAverage;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends TerribleSubsystem {

    public static final double SHOOT_TIME = 3.0;
    public static final double INDEX_TIME = 7.0;

    private static final double READY_SPEED = 0.6;
    private static final double SHOOT_SPEED = 0.5;

    private static final double INDEX_BIAS = 0.1;
    private static final double INDEX_AMP = 0.4;
    private static final double INDEX_PER = 1.0;

    private static final double STANDBY_BIAS = INDEX_BIAS;
    private static final double STANDBY_AMP = INDEX_AMP;
    private static final double STANDBY_PER = 0.8;

    private static final double READY_BIAS = INDEX_BIAS;
    private static final double READY_AMP = INDEX_AMP;
    private static final double READY_PER = INDEX_PER;

    private static final double SHOOT_BIAS = 0.2;
    private static final double SHOOT_AMP = 0.4;
    private static final double SHOOT_PER = 0.4;

    private static final double UNJAM_BIAS = -0.2;
    private static final double UNJAM_AMP = 0.6;
    private static final double UNJAM_PER = 1.0;

    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DIST_P_PULSE = (24.0 / 400.0) / TICKS_PER_ENCODER_REV * 5.0; // Converts to Spindexer Sections

    private WPI_TalonSRX spindexerSRX;

    private enum MODE{
        STOPPED,
        INDEX,
        STANDBY,
        JAM_CLEAR,
        READYING,
        SHOOTING,
    }

    private MODE spinMode = MODE.STOPPED;
    private MODE lastMode = MODE.STOPPED;

    private Encoder spinCoder;

    private MovingAverage jamCounter;
    private static final double MIN_UNJAMMED_SPEED = 1.0;
    private static final double AVG_JAM_THRESHOLD = 0.5;

    public Spindexer() {
        setName("Spindexer");
        spindexerSRX = new WPI_TalonSRX(Constants.Shooter.SPINDEXER_MOTOR_SRX);
        spindexerSRX.setInverted(true);
        spinCoder = new Encoder(0, 1);
        spinCoder.setDistancePerPulse(DIST_P_PULSE);
        jamCounter = new MovingAverage(100, true);
        stopSpinning();
        // updateThreadStart();
    }

    private double getPosition(){
        return spinCoder.getDistance();
    }

    private boolean isJammed() {
        // TODO current using Talon?
        double speed = spinCoder.getRate();
        boolean jammed = Utils.withinThreshold(speed, 0.0, MIN_UNJAMMED_SPEED);
        return jammed && spinMode != MODE.STOPPED;
    }

    private void jamChecker() {
        double avg;
        if (isJammed()) avg = jamCounter.update(1.0);
        else avg = jamCounter.update(0.0);
        boolean jamClear = avg > AVG_JAM_THRESHOLD;
        display("auto jam clear", jamClear);
        if (avg > AVG_JAM_THRESHOLD && spinMode != MODE.JAM_CLEAR)
            CommandScheduler.getInstance().schedule(new StartEndCommand(() -> toggleJamClear(),
                                                                        () -> toggleJamClear(),
                                                                        this)
                                                                    .withTimeout(2.0));
        display("jam avg", avg);
    }


    public boolean isReady(){
        double moddedDelta = spinCoder.getDistance() % 1.0;
        // TODO might have to tune this
        boolean readyState = moddedDelta < 0.1 || moddedDelta > 0.9;
        readyState &= spinMode == MODE.READYING;
        return readyState;
    }

    private void setOutput() {
        switch (spinMode){
            case STOPPED:
                setSpinning(0.0);
                break;
            case INDEX:
                spinBackAndForth(INDEX_BIAS, INDEX_AMP, INDEX_PER);
                break;
            case READYING:
                spinBackAndForth(READY_BIAS, READY_AMP, READY_PER);
                // setSpinning(READY_SPEED);
                break;
            case STANDBY:
                spinBackAndForth(STANDBY_BIAS, STANDBY_AMP, STANDBY_PER);
                // spinBackAndForth(STANDBY_BIAS, STANDBY_AMP, STANDBY_PER);
                break;
            case JAM_CLEAR:
                spinBackAndForth(UNJAM_BIAS, UNJAM_AMP, UNJAM_PER);
                break;
            case SHOOTING:
                // spinBackAndForth(SHOOT_BIAS, SHOOT_AMP, SHOOT_PER);
                setSpinning(SHOOT_SPEED);
                break;
        }
        if (spinMode != MODE.STOPPED) jamChecker();
    }

    public void startIndex(){
        if (spinMode == MODE.STANDBY || spinMode == MODE.STOPPED)
            spinMode = MODE.INDEX;
    }

    public void setStandby() {
        spinMode = MODE.STANDBY;
    }

    public void startReadying(){
        spinMode = MODE.READYING;
    }

    public void startShooting(){
        spinMode = MODE.SHOOTING;
    }

    public void toggleJamClear(){
        if (spinMode == MODE.JAM_CLEAR)
            spinMode = lastMode;
        else {
            lastMode = spinMode;
            spinMode = MODE.JAM_CLEAR;
        }
    }

    private void setSpinning(double speed) {
        // TODO we could smooth this out?
        spindexerSRX.set(ControlMode.PercentOutput, -speed);
    }

    private void spinBackAndForth(double bias, double amplitude, double period) {
        setSpinning(Math.sin(Timer.getFPGATimestamp()*6.28/period) * amplitude + bias);
    }

    public void stopSpinning(){
        spinMode = MODE.STOPPED;
    }

    public void monitor(){
        display("Mode", spinMode.toString());
        display("Position", getPosition());
        display("Speed", spinCoder.getRate());
    }

    public void periodic(){
        setOutput();
        monitor();
    }
}
