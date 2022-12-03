package org.firstinspires.ftc.teamcode.whiteout.RobotSlide;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSlideDriver implements Runnable {

    public RobotSlideHW slideHW = new RobotSlideHW();

    private double delta_x = 0.0;
    private double delta_i = 0.0;
    private boolean is_done = true;
    private double tiltSlideBoxPos = -1.0;
    private double rotateSlideBoxArmPos = -1.0;
    //Thread run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSlideDriver(HardwareMap ahwMap, int threadSleepDelay, boolean enableEncoders) {
        slideHW.init(ahwMap);
        sleepTime = threadSleepDelay;

        if (enableEncoders == true) {
            slideHW.initSlideEncoders();
        }
    }

    public void dropGameElement() {
        double tilt_pos_delta = 0.2;
        //double tilt_arm_pos_delta = 0.3;
        double tilt_arm_pos_delta = 0.4;
        //Tilt box
        slideHW.setSlideServoCurrPos(0, -1 * tilt_pos_delta);

        try {
            //Thread.sleep(500);//was 50ms; we changed it because just 100 milliseconds IS TOO SLOW
            Thread.sleep(200);//was 50ms; we changed it because just 100 milliseconds IS TOO SLOW
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //rotate arm

        slideHW.setSlideServoCurrPos(1, -1 * tilt_arm_pos_delta);
        //slideHW.rotateBoxArm(0.8);

        try {
            Thread.sleep(475);
            //Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Reset Box
        slideHW.setSlideServoCurrPos(0, 1 * tilt_pos_delta);
        /*
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        //Reset Arm
        slideHW.setSlideServoCurrPos(1, 1 * tilt_arm_pos_delta);
        /*
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

    }

    public void checkGamePad(Gamepad gp) {
        //moveRobotSlideBy(10,0);

        if (gp.dpad_left) {
            slideHW.motorSetRawSpeed(1.0);
            //moveRobotSlideBy(20,0);
        } else if (gp.dpad_right) {
            slideHW.motorSetRawSpeed(-0.8);
            //moveRobotSlideBy(-20,0);

        } else if (gp.dpad_up) {
            dropGameElement();
        }
        /*
        else if (gp.y) {
                moveRobotSlideBy(8.5,0);
        }else if (gp.b) {
            moveRobotSlideBy(4.5,0);
        }else if (gp.a) {
            moveRobotSlideBy(2.5,0);
        }

         */
        /*
        else if (gp.left_bumper) {
            moveMagnetServo(1); //Move servo Up

        } else if (gp.right_bumper) {
            moveMagnetServo(0); //Move servo down

        }
        */

        else {

            slideHW.motorSetRawSpeed(0.0);
        }

    }


    //For no slide up and down
    public void checkGamePadX(Gamepad gp) {
        if (gp.left_stick_button) {
            //rotate arm
            slideHW.setSlideServoCurrPos(1, -1 * 1.0);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            slideHW.setSlideServoCurrPos(1, 1 * 1.0);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }


        }
        else if (gp.left_bumper) {
            moveMagnetServo(1); //Move servo Up

        } else if (gp.right_bumper) {
            moveMagnetServo(2); //go to middle pos

        } else if (gp.x) {
            moveMagnetServo(0); //Move servo down
        }else if (gp.y) {
            moveMagnetServo(3); //Move servo down
        }

        /*
        else if (gp.left_bumper) {
            //rotate arm
            slideHW.setSlideServoCurrPosAbs(1, 1 * 0.3);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (gp.right_bumper) {

            slideHW.setSlideServoCurrPosAbs(1, 1 * 0.0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

         */

    }

/*    public void moveSlideToDropPos(int lvl, SlideDirection dir) {
        if (dir == SlideDirection.OUT) {
            if (lvl == 1) {
                //level 1
                moveRobotSlideBy(5, 0);

            } else if (lvl == 2) {
                //level 2
                moveRobotSlideBy(12, -0.1);

            } else {
                //level 3
                moveRobotSlideBy(17, -0.15);
            }
        } else {
            //dir = IN
            if (lvl == 1) {
                //level 1
                moveRobotSlideBy(-5, 0);
            } else if (lvl == 2) {
                //level 2
                moveRobotSlideBy(-12, 0.1);

            } else {
                //level 3
                moveRobotSlideBy(-17, 0.15);
            }
        }

        return;
    }

 */

    private void robotSlidePositionUpdate() {

        if (delta_x != 0 || delta_i != 0 ) {
            //Move the robot Arm only for non-zero values of delta_x, delta_y
            slideHW.moveSlideTo(delta_x);
            //Do not pull Slides up here.
            //slideHW.pullSlideUp(delta_i);
            delta_x = 0;
            delta_i = 0;
            is_done = true;
        }
    }

    public RobotSlideParams getRobotSlideParams() {
        RobotSlideParams param = new RobotSlideParams();
        param.pos = slideHW.getSlideArmInInch();
        param.inclination = slideHW.getSlideCurrIncl();
        param.encoderPosSl1 = slideHW.getSlideEncoderValue(1);
        param.encoderPosSl2 = slideHW.getSlideEncoderValue(2);
        param.servo0Pos = slideHW.getSlideServoCurrPos(0);
        param.servo1Pos = slideHW.getSlideServoCurrPos(1);
        return param;
    }

    public void moveRobotSlideBy(double dx, double di) {
        if (is_done == true) {
            delta_x = dx;
            delta_i = di;
            is_done = false;
        }
    }

    public void moveMagnetServo(int position) {
        double mag_servo_delta = 0.5;
        if (position == 0) {
            //Magnet servo is down
            slideHW.setSlideServoCurrPosAbs(2, 0.0);
        }else if (position == 1) {
            //magnet servo is UP
            slideHW.setSlideServoCurrPosAbs(2, 0.12);
        } else if (position == 2) {
            //magnet servo is UP
            slideHW.setSlideServoCurrPosAbs(2, 0.04);
        }else if (position == 3) {
            //magnet servo is UP
            slideHW.setSlideServoCurrPosAbs(2, 0.08);
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }


    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotSlidePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
