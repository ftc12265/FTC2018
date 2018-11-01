package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;




/**
 * This class defines all the specific hardware for GearHeads robot Tom
 * Please config the following hardware for Tom
 *      Motor Chanel 0:  left drive
 *      Motor Chanel 1:  right drive
 *
 * Robot motion is defined in XYZ three axis motion
 * Moving towards Front target wall the x value increase
 * Moving towards Back target wall the x value decrease
 * Moving towards to Red target wall the y value increase
 * Moving towards to Blue target wall the y value decrease
 * Counter Clockwise rotation the Z value increase
 * Clockwise rotation the Z value decrease
 */

public class Robot_Tom {

   //private members
    private LinearOpMode myOpMode;
    private DcMotor leftMotor;
    private DcMotor rightMotor;


    private double driveAxial = 0; /**positive value drive forward, negative value drive backward,
     Axial is the direction alone the red target or blue target wall*/

    private double driveLateral = 0; /* positive value drive right, negative value drive left, Lateral
     is the direction along the front target or back target wall*/

    private double driveYaw = 0; /* positive value rotate counter clockwise, negative value rotate clockwise*/

    /*constructor*/
    public Robot_Tom(){}

    /*Initialization standard Hardware interface*/
    public void initDrive(LinearOpMode opMode){
        //save reference to hardware map
        myOpMode = opMode;
        //define and initialize Motors
        leftMotor = myOpMode.hardwareMap.get(DcMotor.class,"left drive");
        rightMotor = myOpMode.hardwareMap.get(DcMotor.class,"right drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // positive input rotates counter clockwise
        rightMotor.setDirection(DcMotor.Direction.FORWARD);//positive input rotate counter clockwise
        //use RUN_USING_ENCODER because encoders are installed
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setAxial(double axial) {driveAxial = Range.clip(axial,-1,1); }
    public void setLateral(double lateral){driveLateral = Range.clip(lateral,-1,1);}
    public void setYaw(double yaw){driveYaw = Range.clip(yaw,-1,1);}

    public void manualDrive(){
        //use left joystick to control fwd/backward,right joystick to turn left or rigt
        setAxial(-myOpMode.gamepad1.left_stick_y);//jystick push forward value is negative
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(myOpMode.gamepad1.right_stick_x);


    }

    public void moveRobot(double axial,double lateral,double yaw){
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();

    }
    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions

        double left = driveYaw - driveAxial - (driveLateral * 0.5);
        double right = driveYaw + driveAxial - (driveLateral * 0.5);

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max > 1.0)
        {

            right /= max;
            left /= max;
        }

        // Set drive motor power levels.

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f]", left, right);
    }

    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);

    }





}
