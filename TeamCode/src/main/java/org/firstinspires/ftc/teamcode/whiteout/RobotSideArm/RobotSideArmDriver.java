package org.firstinspires.ftc.teamcode.whiteout.RobotSideArm;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSideArmDriver implements Runnable {
    public enum RobotSideArmPreSetPos {
        DOWN,
        UP,
        INVALID
    }
    public enum RobotSideArmSide {
        LEFT_SIDE,
        RIGHT_SIDE,
        BOTH_SIDES,
        INVALID

    }
    private RobotSideArmHW sideArmHW = new RobotSideArmHW();
    private RobotSideArmPreSetPos sideArmPos;
    private RobotSideArmSide sideArmSide;
    private boolean is_done = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSideArmDriver(HardwareMap ahwMap, int threadSleepDelay) {
        sideArmHW.init(ahwMap);
        sleepTime = threadSleepDelay;
        sideArmPos = RobotSideArmPreSetPos.UP;
        sideArmSide = RobotSideArmSide.BOTH_SIDES;
    }

    private void robotArmPositionUpdate() {
        if (is_done == false) {
            is_done = true;
            if (sideArmPos == RobotSideArmPreSetPos.UP) {
                if (sideArmSide == RobotSideArmSide.LEFT_SIDE || sideArmSide == RobotSideArmSide.BOTH_SIDES) {
                    sideArmHW.servoSetPosRaw(0, 0.0);
                    try {
                        Thread.sleep(400);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                if (sideArmSide == RobotSideArmSide.RIGHT_SIDE || sideArmSide == RobotSideArmSide.BOTH_SIDES) {
                    sideArmHW.servoSetPosRaw(1, 0.0);
                    try {
                        Thread.sleep(400);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

            } else {
                //DOWN
                if (sideArmSide == RobotSideArmSide.LEFT_SIDE || sideArmSide == RobotSideArmSide.BOTH_SIDES) {
                    sideArmHW.servoSetPosRaw(0, 0.7);
                    try {
                        Thread.sleep(400);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                if (sideArmSide == RobotSideArmSide.RIGHT_SIDE || sideArmSide == RobotSideArmSide.BOTH_SIDES) {
                    sideArmHW.servoSetPosRaw(1, 0.7);
                    try {
                        Thread.sleep(400);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }

        }
    }

    public void checkGamePad(Gamepad gp) {
    }


    public void activateSideArms(RobotSideArmSide side, RobotSideArmPreSetPos pos) {
        if (is_done == true) {
            is_done = false;
            sideArmPos = pos;
            sideArmSide = side;
        }
    }

    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotArmPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
