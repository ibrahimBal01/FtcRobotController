package org.firstinspires.ftc.teamcode.aiming;
import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.ShootingConfig.HoodConfig;
import org.firstinspires.ftc.teamcode.ShootingConfig.TurretConfig;

public class ReviewRegressionTest {
    private HoodConfig hoodConfig() {
        HoodConfig c=new HoodConfig(); c.calibrated=true;
        c.minDeg=20; c.maxDeg=65; c.secondsPerDegree=.02; c.settleS=.3;
        return c;
    }
    @Test public void hoodUnknownPositionNeedsFullTravelTime() {
        HoodController h=new HoodController(hoodConfig());
        assertTrue(Double.isFinite(h.request(30,0)));
        assertFalse(h.atTarget(.5)); assertTrue(h.atTarget(1.21));
    }
    @Test public void hoodRetargetCannotEraseOutstandingTravel() {
        HoodController h=new HoodController(hoodConfig());
        h.request(20,0); h.request(65,.01); h.request(64,.02);
        assertFalse(h.atTarget(.5)); assertFalse(h.atTarget(1.5));
        assertTrue(h.atTarget(2.2));
    }
    @Test public void repeatedSameTargetDoesNotExtendDeadline() {
        HoodController h=new HoodController(hoodConfig());
        h.request(35,0);
        for(int i=1;i<20;i++)h.request(35,i*.1);
        assertTrue(h.atTarget(2));
    }
    @Test public void hoodDeadbandDoesNotSendUnmodelledTravel() {
        HoodController h=new HoodController(hoodConfig());
        double p=h.request(35,0);
        assertEquals(p,h.request(35.04,.1),0);
        assertEquals(35,h.targetDegrees(),0);
    }
    @Test public void hoodCalibrationChangeInvalidatesReady() {
        HoodConfig c=hoodConfig();HoodController h=new HoodController(c);
        h.request(35,0);assertTrue(h.atTarget(2));
        c.positionAtMax=.9;assertFalse(h.atTarget(2));
        h.request(35,2);assertFalse(h.atTarget(2.1));assertTrue(h.atTarget(3.3));
    }
    @Test public void hoodStopAndInvalidRequestsRequireFreshReadiness() {
        HoodController h=new HoodController(hoodConfig());
        h.request(35,0);assertTrue(h.atTarget(2));h.stop();assertFalse(h.atTarget(3));
        h.request(35,3);assertFalse(h.atTarget(3.1));
        assertTrue(Double.isNaN(h.request(Double.NaN,4)));assertFalse(h.atTarget(5));
    }
    @Test public void hoodRejectsCollapsedMappingAndZeroTravelTime() {
        HoodConfig c=hoodConfig();c.positionAtMax=c.positionAtMin;
        HoodController h=new HoodController(c);assertTrue(Double.isNaN(h.request(35,0)));
        c.positionAtMax=.8;c.secondsPerDegree=0;
        assertTrue(Double.isNaN(h.request(35,0)));
    }
    @Test public void hoodAllowsReversedServoMapping() {
        HoodConfig c=hoodConfig();c.positionAtMin=.8;c.positionAtMax=.2;
        HoodController h=new HoodController(c);assertEquals(.5,h.request(42.5,0),1e-8);
    }
    @Test public void latestMovingSamplePreventsFeedBeforeTrackExecutes() {
        MotionGate g=new MotionGate();
        assertFalse(g.update(0,0,0,0,0,2,5));
        assertTrue(g.update(0,0,0,.02,.02,2,5));
        assertFalse(g.update(1,0,0,.04,.04,2,5));
        // The tracking command seeing the same sample must not turn this into a zero-speed sample.
        assertFalse(g.update(1,0,0,.04,.041,2,5));
    }
    @Test public void duplicateStationarySampleStaysReady() {
        MotionGate g=new MotionGate();g.update(0,0,0,0,0,2,5);
        assertTrue(g.update(0,0,0,.02,.02,2,5));
        assertTrue(g.update(0,0,0,.02,.025,2,5));
    }
    @Test public void staleAndResetPoseInvalidateMotionGate() {
        MotionGate g=new MotionGate();g.update(0,0,0,0,0,2,5);
        assertTrue(g.update(0,0,0,.02,.02,2,5));
        assertFalse(g.update(0,0,0,.02,.3,2,5));
        g.reset();g.update(0,0,0,0,0,2,5);g.update(0,0,0,.02,.02,2,5);
        assertFalse(g.update(100,0,0,.02,.03,2,5));
    }
    @Test public void rotationAndNonfiniteLimitsPreventFeed() {
        MotionGate g=new MotionGate();g.update(0,0,0,0,0,2,5);
        assertFalse(g.update(0,0,Math.toRadians(5),.02,.02,2,5));
        assertFalse(g.update(0,0,0,.04,.04,Double.POSITIVE_INFINITY,5));
    }
    @Test public void motionGateUsesShortestHeadingDifference() {
        MotionGate g=new MotionGate();g.update(0,0,Math.toRadians(179.99),0,0,2,5);
        assertTrue(g.update(0,0,Math.toRadians(-179.99),.02,.02,2,5));
    }
    private TurretController home(TurretConfig c) {
        c.calibrated=true;TurretController t=new TurretController(c);
        t.home(0,false);t.update(0,true,.01);t.update(0,true,.06);
        assertEquals(TurretController.State.HOLD,t.state());return t;
    }
    @Test public void changingEncoderScaleRequiresRehome() {
        TurretConfig c=new TurretConfig();TurretController t=home(c);
        t.aim(c.minDeg);assertTrue(t.atTarget());
        c.encoderTurnsPerTurretTurn=3;assertFalse(t.atTarget());assertFalse(t.aim(0));
        assertEquals(0,t.update(0,false,.08),0);
        assertEquals(TurretController.State.FAULT,t.state());
    }
    @Test public void changingReferenceAngleCanBeRecoveredByRehome() {
        TurretConfig c=new TurretConfig();TurretController t=home(c);
        c.minDeg=-80;t.home(.07,false);t.update(0,true,.08);t.update(0,true,.13);
        assertEquals(TurretController.State.HOLD,t.state());assertEquals(-80,t.angle(),0);
    }
    @Test public void turretRejectsNegativeGainsAndNonfiniteReadinessLimit() {
        TurretConfig c=new TurretConfig();c.calibrated=true;c.kP=-1;
        assertFalse(new TurretController(c).validConfig());
        c.kP=.01;c.maxReadyVelocityDegS=Double.POSITIVE_INFINITY;
        assertFalse(new TurretController(c).validConfig());
    }
    @Test(timeout=1000) public void hugeAngleBoundsDoNotOverflowIntegerLoop() {
        double min=360.0*Integer.MAX_VALUE,max=min+360;
        assertEquals(min,AimMath.reachableAngle(0,min,min,max),1e-4);
    }
}

