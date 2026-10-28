package org.firstinspires.ftc.teamcode.aiming;
import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.ShootingConfig.TurretConfig;

public class AimingTest {
    @Test public void rotatesMountOffsetWithRobot() {
        double[] a=AimMath.solve(10,20,Math.PI/2,4,2,0,8,30,0);
        assertEquals(8,a[2],1e-8); assertEquals(24,a[3],1e-8);
        assertEquals(0,a[0],1e-8); assertEquals(6,a[1],1e-8);
    }
    @Test public void accountsForMuzzleDistanceAndYawOffset() {
        double[] a=AimMath.solve(0,0,0,0,0,15,100,0,5);
        assertEquals(-15,a[0],1e-8); assertEquals(95,a[1],1e-8);
    }
    @Test public void neverClampsUnreachableTarget() {
        assertTrue(Double.isNaN(AimMath.reachableAngle(180,0,-90,90)));
    }
    @Test public void choosesAllowedEquivalentAcrossWrap() {
        assertEquals(190,AimMath.reachableAngle(-170,185,0,270),1e-8);
    }
    @Test public void respectsCableBoundaryEvenWhenLongWayRequired() {
        assertEquals(-170,AimMath.reachableAngle(-170,170,-175,175),1e-8);
    }
    @Test public void interpolationAndEndpoints() {
        double[][] t={{30,2000,25},{60,3000,45}};
        assertArrayEquals(new double[]{2500,35},ShotTable.lookup(t,45),1e-8);
        assertArrayEquals(new double[]{3000,45},ShotTable.lookup(t,60),1e-8);
    }
    @Test public void rejectsUncalibratedAndOutOfRangeShots() {
        assertNull(ShotTable.lookup(new double[0][3],30));
        assertNull(ShotTable.lookup(new double[][]{{30,2000,25},{60,3000,45}},61));
        assertNull(ShotTable.lookup(new double[][]{{30,2000,25},{60,3000,45}},29));
    }
    @Test public void rejectsBadTableData() {
        assertNull(ShotTable.lookup(new double[][]{{30,2000,25},{30,3000,45}},30));
        assertNull(ShotTable.lookup(new double[][]{{30,Double.NaN,25},{60,3000,45}},40));
        assertNull(ShotTable.lookup(new double[][]{{30,-1,25},{60,3000,45}},40));
    }
    private TurretConfig config() {
        TurretConfig c=new TurretConfig(); c.calibrated=true; return c;
    }
    private TurretController homed(TurretConfig c) {
        TurretController t=new TurretController(c); t.update(0,false,0);
        t.home(.01,false); assertTrue(t.update(0,false,.02)<0);
        assertEquals(0,t.update(0,true,.04),0); t.update(0,true,.09);
        assertEquals(TurretController.State.HOLD,t.state()); return t;
    }
    @Test public void uncalibratedNeverMoves() {
        TurretConfig c=config(); c.calibrated=false;
        TurretController t=new TurretController(c); t.home(0,false);
        assertEquals(0,t.update(0,false,.02),0); assertFalse(t.aim(0));
    }
    @Test public void switchMustBeDebouncedBeforeZeroing() {
        TurretConfig c=config(); TurretController t=new TurretController(c);
        t.home(0,false); t.update(0,true,.01); t.update(0,false,.02);
        assertEquals(TurretController.State.SEEK_HOME,t.state());
        t.update(0,true,.03); t.update(0,true,.08);
        assertEquals(TurretController.State.HOLD,t.state()); assertEquals(c.minDeg,t.angle(),1e-8);
    }
    @Test public void clearsInitiallyActiveSwitchBeforeSeeking() {
        TurretController t=new TurretController(config()); t.home(0,true);
        assertTrue(t.update(0,true,.01)>0);
        t.update(1,false,.02); assertTrue(t.update(2,false,.07)<0);
        assertEquals(TurretController.State.SEEK_HOME,t.state());
    }
    @Test public void interruptingHomeRequiresRehome() {
        TurretController t=new TurretController(config()); t.home(0,false); t.stop();
        assertEquals(TurretController.State.UNHOMED,t.state()); assertFalse(t.aim(0));
    }
    @Test public void encoderRatioIsIndependentOfServoRatio() {
        TurretConfig c=config(); c.encoderTurnsPerTurretTurn=3; c.maxMeasuredVelocityDegS=5000;
        TurretController t=homed(c); t.update(8192,false,.19);
        assertEquals(c.minDeg+120,t.angle(),1e-8);
    }
    @Test public void signedEncoderAndSoftLimit() {
        TurretConfig c=config(); c.encoderSign=-1;
        TurretController t=homed(c); assertTrue(t.aim(-140));
        assertTrue(t.update(-c.ticksPerEncoderTurn*5/360,false,.19)>0);
        assertEquals(-145,t.angle(),1e-6);
        assertFalse(t.aim(180));
    }
    @Test public void noMovementUnderPowerFaults() {
        TurretController t=homed(config()); t.aim(0);
        for(int i=1;i<=8;i++)t.update(0,false,.09+i*.1);
        assertEquals(TurretController.State.FAULT,t.state());
    }
    @Test public void homeTimeoutStopsMotor() {
        TurretConfig c=config(); c.homeTimeoutS=.25;
        TurretController t=new TurretController(c); t.home(0,false);
        t.update(0,false,.01); t.update(-50,false,.11); t.update(-100,false,.21);
        assertEquals(0,t.update(-150,false,.31),0);
        assertEquals(TurretController.State.FAULT,t.state());
    }
    @Test public void unexpectedSwitchAndLoopGapFault() {
        TurretController t=homed(config()); t.update(500,false,.19);
        assertEquals(0,t.update(500,true,.29),0); assertEquals(TurretController.State.FAULT,t.state());
        t=homed(config()); assertEquals(0,t.update(0,false,1),0);
        assertEquals(TurretController.State.FAULT,t.state());
    }
    @Test public void pulseNeedsReadinessAndFinishesOnce() {
        FeedCycle c=new FeedCycle();c.start(0);
        assertFalse(c.update(.1,false,true,2,.2));
        assertTrue(c.update(.2,true,true,2,.2));
        // Expected RPM dip after release does not truncate the pulse if geometry remains safe.
        assertTrue(c.update(.3,false,true,2,.2));
        assertFalse(c.update(.41,true,true,2,.2));assertEquals(FeedCycle.State.DONE,c.state());
        assertFalse(c.update(.5,true,true,2,.2));
    }
    @Test public void pulseAbortsOnLostAimAndCannotResume() {
        FeedCycle c=new FeedCycle();c.start(0);c.update(.1,true,true,2,.2);
        assertFalse(c.update(.15,true,false,2,.2));
        assertFalse(c.update(.16,true,true,2,.2));assertEquals(FeedCycle.State.ABORTED,c.state());
    }
    @Test public void queuedShotExpires() {
        FeedCycle c=new FeedCycle();c.start(0);
        assertFalse(c.update(2,true,true,2,.2));assertEquals(FeedCycle.State.ABORTED,c.state());
    }
    @Test public void selectsOwnAllianceAndEntranceSide() {
        double[] old=ShootingConfig.RED_AUDIENCE;
        try {
            ShootingConfig.RED_AUDIENCE=new double[]{60,55,58};
            FieldTargets f=new FieldTargets();f.select(FieldTargets.Alliance.RED,FieldTargets.End.AUDIENCE);
            assertArrayEquals(new double[]{60,55,58},f.point(),0);
            assertTrue(f.onEntranceSide(60,40)); assertFalse(f.onEntranceSide(60,70));
        } finally { ShootingConfig.RED_AUDIENCE=old; }
    }
}

