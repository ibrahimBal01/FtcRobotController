package org.firstinspires.ftc.teamcode.aiming;
/** One rising-edge request, one bounded pulse; no queued shot survives timeout/cancellation. */
public final class FeedCycle {
    public enum State { WAITING, OPEN, DONE, ABORTED }
    private State state=State.DONE;
    private double start, opened;
    public void start(double now) { start=now; state=State.WAITING; }
    public boolean update(double now,boolean ready,boolean safe,double wait,double pulse) {
        if (!AimMath.finite(now,wait,pulse) || wait<=0 || pulse<=0) { state=State.ABORTED; return false; }
        if(state==State.WAITING) {
            if(now-start>=wait) state=State.ABORTED;
            else if(ready && safe) { state=State.OPEN; opened=now; }
        }
        if(state==State.OPEN) {
            if(!safe) state=State.ABORTED;
            else if(now-opened>=pulse) state=State.DONE;
        }
        return state==State.OPEN;
    }
    public State state() { return state; }
    public boolean finished() { return state==State.DONE || state==State.ABORTED; }
}

