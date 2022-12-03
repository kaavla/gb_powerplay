package org.firstinspires.ftc.teamcode.whiteout.RobotArm;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotArmDriver implements Runnable {
    public enum RobotArmPreSetPos {
        COLLECT,
        SAVE,
        DROP,
        INVALID
    }
    private RobotArmHW armHW = new RobotArmHW();

    private double delta_x = 0.0;
    private double delta_y = 0.0;
    private double delta_theta = 0.0;
    private RobotArmPreSetPos armPos = RobotArmPreSetPos.INVALID;
    private boolean is_done = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotArmDriver(HardwareMap ahwMap, int threadSleepDelay) {
        armHW.init(ahwMap);
        sleepTime = threadSleepDelay;
    }
    public void initArmPos(RobotArmPreSetPos pos) {
        if (pos == RobotArmPreSetPos.COLLECT) {
            //Collect position
            //arm servo
            armHW.servoSetPosRaw(0.3, 2);
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //base servo
            armHW.servoSetPosRaw(0.9, 1);
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            armHW.servoSetPosRaw(0.5, 3); //0.6 pos

        }  else if (pos == RobotArmPreSetPos.SAVE) {
            //Save position

            //base servo
            armHW.servoSetPosRaw(0.1, 1);//0.2 pos

            //arm servo
            armHW.servoSetPosRaw(0.7, 2);//0.5
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //servo at the end of the arm
            armHW.servoSetPosRaw(0.6, 3);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else if (pos == RobotArmPreSetPos.DROP) {
            //drop position
            //arm servo
            armHW.servoSetPosRaw(0.0, 2);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //base servo
            armHW.servoSetPosRaw(0.5, 1);
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            armHW.servoSetPosRaw(1.0, 3);

        }

    }

    private void robotArmPositionUpdate() {
        double xpos = armHW.getArmTipXPos();
        double ypos = armHW.getArmTipYPos();

        if (delta_x != 0 || delta_y != 0) {
            //Move the robot Arm only for non-zero values of delta_x, delta_y
            armHW.moveRobotArmTo(xpos + delta_x, ypos + delta_y);

            //reset delta_x/delta_y state
            delta_x = 0;
            delta_y = 0;
            is_done = true;
        }
        if (delta_theta != 0) {
            armHW.rotateArmTo(delta_theta);

            delta_theta = 0.0;
            is_done = true;
        }
        if (armPos != RobotArmPreSetPos.INVALID) {
            initArmPos(armPos);
            armPos = RobotArmPreSetPos.INVALID;
            is_done = true;
        }

    }

    public void checkGamePad(Gamepad gp) {
        if ((gp.left_stick_y != 0) || (gp.left_stick_x != 0)) {
            double leftY = gp.left_stick_y * -1;
            double leftX = gp.left_stick_x * 1;
            moveRobotArmBy(leftX, leftY, 0.0);
        } else if (gp.left_bumper) {
            armHW.servoSetPosRaw(0.7, 3);
        } else if (gp.right_bumper) {
            armHW.servoSetPosRaw(1.5,3);
        } else if (gp.left_stick_button) {
            moveRobotArmBy(0.0,0.0, 15);
        } else if (gp.right_stick_button) {
            moveRobotArmBy(0,0.0, -15);
        } else if (gp.x) {
            //Collect
            moveRobotArmTo(RobotArmPreSetPos.COLLECT);
        } else if (gp.y) {
            moveRobotArmTo(RobotArmPreSetPos.SAVE);

        } else if (gp.a) {
            moveRobotArmTo(RobotArmPreSetPos.DROP);
        }
    }



    public RobotArmParams getRobotParams() {
        RobotArmParams param = new RobotArmParams();
        param.xpos = armHW.getArmTipXPos();
        param.ypos = armHW.getArmTipYPos();
        param.baseServoAbsPosInDegree = armHW.getBaseAbsPosInDegree();
        param.linkServoAbsPosInDegree = armHW.getLinkAbsPosInDegree();
        param.currOrientationInDegree = armHW.getCurrOrientationInDegree();
        param.motorEncoderRawVal = armHW.getMotorEncoderRawValue();
        param.s1ServoRawPos = armHW.getServoPosRaw(1);
        param.s2ServoRawPos = armHW.getServoPosRaw(2);
        param.s3ServoRawPos = armHW.getServoPosRaw(3);

        return param;
    }

    public void moveRobotArmBy(double dx, double dy, double d_theta) {
        if (is_done == true) {
            delta_x = dx;
            delta_y = dy;
            delta_theta = d_theta;
            is_done = false;
        }
    }

    public void moveRobotArmTo(RobotArmPreSetPos pos) {
        if (is_done == true) {
            armPos = pos;
            is_done = false;
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
