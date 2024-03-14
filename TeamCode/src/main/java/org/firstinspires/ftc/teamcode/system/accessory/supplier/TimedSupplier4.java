package org.firstinspires.ftc.teamcode.system.accessory.supplier;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class TimedSupplier4<T> {
    private final AtomicReference<T> value;
    private long period;
    protected final AtomicLong timer;
    protected final AtomicBoolean running;
    private final FutureSupplier<T> supplier;

    // to be called in init
    public TimedSupplier4(FutureSupplier<T> supplier, long period){
        this.period = period;
        timer = new AtomicLong(System.currentTimeMillis() + period);
        running = new AtomicBoolean(false);
        value = new AtomicReference<>();
        this.supplier = supplier;
        this.period = period;
    }

    public void update(){
        // run update every cycle, the timer should make it periodic
        if(!running.get() && System.currentTimeMillis() > timer.get()){
            // use of future to run an async operation that should be faster then multi threading
            CompletableFuture<T> future = supplier.getSupplier();
            running.set(true); // don't really know if this is necessary but should keep calls from stacking
            future.thenAccept((result) -> {
                this.value.set(result);
                running.set(false);
                //.set(System.currentTimeMillis() + period);

            });
        }
    }

    // use this to call for the value of the sensor, it returns the cached value if the call is in idle in the different thread
    public T get(){
        return value.get();
    }

    @FunctionalInterface
    public interface FutureSupplier<T> {
        CompletableFuture<T> getSupplier();
    }

    // the idea if the bus is hijacked by the call when at low can i asynchronously call it let it run and then get it back at high


}