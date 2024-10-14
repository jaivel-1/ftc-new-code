package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.Pipelines.WebcamPipeline;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.PixelDropper;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class StageRedWall extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);
        PixelDropper pixelDropper = new PixelDropper(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.RED_STAGE);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        // Initialize the sub systems. Note the init method is inside the subsystem class
        pixelDropper.init(hardwareMap);
        lift.init(hardwareMap);
        //sleep(500);

        // Set start positions
        lift.setAnglerLoad();
        sleep(250);
        lift.gripperClosed();
        pixelDropper.dropperClosed();
        lift.slideMechanicalReset();
        lift.setAnglerCarry();
        sleep(250); // no sleepy no workie. Need this to let the anger servo have time to move

        waitForStart();

        detector.toggleTelemetry();
        telemetry.clearAll();

        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //============================
        // POSE SECTION
        //============================
        // Common poses for all 3 Red Stage Prop Positions
        Pose2d startPos = new Pose2d(62.5, 12, Math.toRadians(90));
        Pose2d RedPark = new Pose2d(64,50,Math.toRadians(90));

        // Center Prop
        // pixel drop point
        Pose2d StageRedCenter = new Pose2d(22,12,Math.toRadians(90));
        //backstage drop
        Pose2d StageRedCenterDropoff = new Pose2d(39.5, 54, Math.toRadians(90));

        // Left Prop Poses - this one has an extra motion
        // Strafe and rotate towards drive team.
        Pose2d StageRedLeft1 = new Pose2d(32,18, Math.toRadians(0));
        // Strafe under the truss partially to drop the pixel
        Pose2d StageRedLeft2 = new Pose2d(32,-5.5,Math.toRadians(0));
        // Position on the backstage board to drop yellow pixel
        Pose2d StageRedLeftDropoff = new Pose2d(24,54, Math.toRadians(90));

        // Right Prop Poses

        Pose2d StageRedRight = new Pose2d(29,25,Math.toRadians(90));
        Pose2d StageRedRightDropoff = new Pose2d(47.5, 54, Math.toRadians(90));


        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageRedLeft1)
                .lineToLinearHeading(StageRedLeft2)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.5)
                .lineToLinearHeading(StageRedLeft1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageRedLeftDropoff)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.25)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperClosed();})
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()-> lift.gripperClosed())
                .strafeLeft(0.5)
                .build();

        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageRedCenter)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.5)
                .strafeLeft(16)
                .lineToLinearHeading(StageRedCenterDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.25)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperClosed();})
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()-> lift.gripperClosed())
                .strafeLeft(0.5)
                .build();

        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageRedRight)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.5)
                .strafeLeft(16)
                .lineToLinearHeading(StageRedRightDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.25)
                .UNSTABLE_addDisplacementMarkerOffset(0.0, ()->{pixelDropper.dropperClosed();})
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()-> lift.gripperClosed())
                .strafeLeft(0.5)
                .build();

        detector.toggleTelemetry();
        telemetry.clearAll();

        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " RED_STAGE" + location);
        telemetry.update();

        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(StageRedLeftTraj1);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageRedCenterTraj1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageRedRightTraj1);
                break;
            default:
                throw new IllegalArgumentException("The code is most certainly severely screwed up.");
        }
    }
}
