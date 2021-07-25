/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends TerribleSubsystem {

    public static final double SHOOT_TIME = 3.0;
    public static final double INDEX_TIME = 5.0;
    private static final double READY_SPEED = 0.2;
    private static final double INDEX_SPEED = 0.3;
    private static final double SHOOT_SPEED = 0.5;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DIST_P_PULSE = (24.0 / 400.0) / TICKS_PER_ENCODER_REV * 5.0; // Converts to Spindexer Sections
    private WPI_VictorSPX spindexerSPX;

    private enum MODE{
        STOPPED,
        INDEX,
        JAM_CLEAR,
        READYING,
        SHOOTING,
        MOVE_ONE
    }

    private MODE spinMode = MODE.STOPPED;
    private MODE lastMode = MODE.STOPPED;

    //private DigitalInput spindexerInPosition; //, ballSensor2, ballSensor3, ballSensor4,
    // ballSensor5;
    private Encoder spinCoder;

    private double jamPosition = 0.0;
    private double moveOneTracker = 0.0;
    private boolean jamFlipper = false;
    private int flipDebounceCounter = 0;

    public Spindexer() {
        setName("Spindexer");
        spindexerSPX = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        spindexerSPX.setInverted(true);
        spinCoder = new Encoder(0, 1);
        spinCoder.setDistancePerPulse(DIST_P_PULSE);
    }

    public double getPosition(){
        return spinCoder.getDistance();
    }

    public boolean clearingJam(){
        if (spinMode == MODE.JAM_CLEAR) return true;
        else return false;
    }


    public boolean isReady(){
        boolean readyState = false;
        double moddedDelta = spinCoder.getDistance() % 1.0;
        if (moddedDelta < 0.1 || moddedDelta > 0.9) readyState = true;
        return readyState;
    }

    private void setOutput() {
        switch (spinMode){
            case STOPPED:
                setSpinning(0.0);
                break;
            case INDEX:
                setSpinning(INDEX_SPEED);
                break;
            case READYING:
                if (!isReady()) setSpinning(READY_SPEED);
                else {
                    spinMode = MODE.STOPPED;
                    setSpinning(0.0);
                }
                break;
            case JAM_CLEAR:
                clearJam();
                break;
            case SHOOTING:
                setSpinning(SHOOT_SPEED);
                break;
            case MOVE_ONE:
                if (getPosition() < moveOneTracker+0.9) setSpinning(INDEX_SPEED);
                else {
                    spinMode = MODE.STOPPED;
                    setSpinning(0.0);
                }
        }
    }

    public void startIndex(){
        spinMode = MODE.INDEX;
    }

    public void startReadying(){
        spinMode = MODE.READYING;
    }

    public void startShooting(){
        spinMode = MODE.SHOOTING;
    }

    public void startJamClear(){
        lastMode = spinMode;
        spinMode = MODE.JAM_CLEAR;
        jamPosition = getPosition();
    }

    public void startMoveOne(){
        spinMode = MODE.MOVE_ONE;
        moveOneTracker = getPosition();
    }

    public void clearedJam(){
        spinMode = lastMode;
    }

    private void clearJam(){
        double output = 0.35;
        boolean currentFlip = jamFlipper;
        if (Math.abs(getPosition()-jamPosition) > 0.25 && flipDebounceCounter == 0) jamFlipper = jamFlipper^true;
        if (flipDebounceCounter > 0) flipDebounceCounter += 1;
        if (flipDebounceCounter > 25) flipDebounceCounter = 0;
        if (jamFlipper != currentFlip) flipDebounceCounter = 1;
        if (jamFlipper) output *= -1;
        setSpinning(output);
    }

    private void setSpinning(double speed) {
        spindexerSPX.set(ControlMode.PercentOutput, speed);
    }

    public void stopSpinning(){
        spinMode = MODE.STOPPED;
    }

    public boolean isStopped() {
        return spinMode == MODE.STOPPED;
    }

    public void monitor(){
        SmartDashboard.putString("Spindexer/Spindexer Mode", spinMode.toString());
        display("Position", getPosition());
        display("Slot", Math.round((getPosition() % 5)+1));
        display("Output", spindexerSPX.getMotorOutputPercent());
        display("Int Cast", (int)getPosition());
    }


    public void periodic(){
        setOutput();
    }
}
