package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class ArmRobotUtilities
{
    private static DcMotor leftArmMotor1 = null;
    public static DcMotor leftArmMotor2 = null;
    public static DcMotor rightArmMotor1 = null;
    public static DcMotor rightArmMotor2 = null;
    public static Servo gripper = null;
    public static Servo wrist = null;
    public static ElapsedTime runtime = new ElapsedTime();


    // Gripper & Wrist Ranges:
    public static final double gripperClosedPosition = 0.6;
    public static final double gripperOpenPosition = 0.40;
    public static final double wristUpPosition = 1.5;
    public static final double wristDownPosition = 0.49;// 0.47 0.46

    public static final int armHomePosition = 0;
    public static final int armIntakePosition = 0;
    public static final int armIntermediate = 300;
    public static final int armScoreLeftPosition = 525;
    public static final double armSpeed=0.3;
    public static final int armShutdownThreshold = 5;


    public static void InitializeArm(LinearOpMode opMode)
    {
        leftArmMotor1 = opMode.hardwareMap.get(DcMotor.class, "leftArmMotor1");
        leftArmMotor2 =  opMode.hardwareMap.get(DcMotor.class, "leftArmMotor2");
        rightArmMotor1 =  opMode.hardwareMap.get(DcMotor.class, "rightArmMotor1");
        rightArmMotor2 =  opMode.hardwareMap.get(DcMotor.class, "rightArmMotor2");
        gripper = opMode.hardwareMap.get(Servo.class, "gripper");
        wrist =  opMode.hardwareMap.get(Servo.class, "wrist");

        leftArmMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor2.setDirection(DcMotor.Direction.REVERSE);

        rightArmMotor1.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }
    public static void ResetArm()
    {
        runtime.reset();



        //armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armLeft.setTargetPosition(armHomePosition);
        leftArmMotor1.setTargetPosition(armHomePosition);
        leftArmMotor2.setTargetPosition(armHomePosition);
        //armRight.setTargetPosition(armHomePosition);
        rightArmMotor1.setTargetPosition(armHomePosition);
        rightArmMotor2.setTargetPosition(armHomePosition);
        //armLeft.setPower(1.0);
        leftArmMotor1.setPower(1.0);
        leftArmMotor2.setPower(1.0);
        //armRight.setPower(1.0);
        rightArmMotor1.setPower(1.0);
        rightArmMotor2.setPower(1.0);
        //armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armLeft.setPower(0);
        leftArmMotor1.setPower(0);
        leftArmMotor2.setPower(0);
        //armRight.setPower(0);
        rightArmMotor1.setPower(0);
        rightArmMotor2.setPower(0);
    }

    public static void encoderArm( double speed,int EncoderTarget,
                            double timeoutS, LinearOpMode opMode)
    {

        runtime.reset();
        leftArmMotor1.setTargetPosition(EncoderTarget);
        leftArmMotor2.setTargetPosition(EncoderTarget);
        rightArmMotor1.setTargetPosition(EncoderTarget);
        rightArmMotor2.setTargetPosition(EncoderTarget);

        leftArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor1.setPower(speed);
        leftArmMotor2.setPower(speed);
        rightArmMotor1.setPower(speed);
        rightArmMotor2.setPower(speed);


        while ((leftArmMotor1.isBusy() || leftArmMotor2.isBusy() ||rightArmMotor1.isBusy()
                ||rightArmMotor2.isBusy()) &&(runtime.seconds() < timeoutS) && opMode.opModeIsActive()
        &&  !opMode.isStopRequested() )
        {
            /*leftArmMotor1.setTargetPosition(EncoderTarget);
            leftArmMotor2.setTargetPosition(EncoderTarget);

            rightArmMotor1.setTargetPosition(EncoderTarget);
            rightArmMotor2.setTargetPosition(EncoderTarget);

            leftArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftArmMotor1.setPower(speed);
            leftArmMotor1.setPower(speed);

            rightArmMotor1.setPower(speed);
            rightArmMotor2.setPower(speed);*/

            // Display it for the driver.
            opMode.telemetry.addData("arm Running to", "left %7d and right %7d ",
                    EncoderTarget,-EncoderTarget);
            opMode.telemetry.addData("arm Left Currently at", " at %7d",
                    leftArmMotor1.getCurrentPosition());
            opMode.telemetry.addData("arm Left Currently at", " at %7d",
                    leftArmMotor2.getCurrentPosition());
            opMode.telemetry.addData("arm Right Currently at", " at %7d",
                    rightArmMotor1.getCurrentPosition());
            opMode.telemetry.addData("arm Right Currently at", " at %7d",
                    rightArmMotor2.getCurrentPosition());
            opMode.telemetry.update();
        }

    }

    public static void OperateGripper(double gripperSetPosition)
    {
        runtime.reset();
        // open gripper to drop pixel
        do {
            gripper.setPosition(gripperSetPosition);// Open Gripper to drop of pixel
        } while (gripper.getPosition() != gripperSetPosition | (runtime.seconds() < 0.3));
       // sleep(100);

    }

    public static void OperateWrist ( double wristSetPosition)
    {
        wrist.setPosition(wristSetPosition);

    }





}
