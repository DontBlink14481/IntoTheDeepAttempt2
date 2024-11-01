package org.firstinspires.ftc.teamcode.Util;

import java.util.LinkedList;
import java.util.Queue;

@FunctionalInterface
interface SchedulerMethod {
    void run();
}

@FunctionalInterface
interface ComputingMethod{
    void compute();
}



public class TimeScheduler {
    private Queue<SchedulerMethod> queue = new LinkedList<SchedulerMethod>();
    public TimeScheduler(){

    }

    public void add(SchedulerMethod method){
        queue.add(method);
    }

    public void run(long nanoseconds){
        if(queue.peek() == null) return;
        long startTime = System.nanoTime();
        while(System.nanoTime() - startTime < nanoseconds){
            queue.peek().run();
            queue.poll();
        }
    }

    public static class QueueThread extends Thread{

    }

    public void update(){
        Thread t = new QueueThread();
        t.stop();
    }


}
