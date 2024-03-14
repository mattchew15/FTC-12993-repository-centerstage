package org.firstinspires.ftc.teamcode.system.accessory.supplier;


import java.util.function.Supplier;

public class TimedSupplier<T> implements Supplier<T>
{
    private final Supplier<T> supplier;
    private T cached = null;
    private final double period;
    private long timer;
    private boolean running;

    public TimedSupplier(Supplier<T> supplier, double period)
    {
        this.supplier = supplier;
        this.period = period;
        this.timer = System.currentTimeMillis();
        this.running = false;

    }

    @Override
    public T get() {
        if (System.currentTimeMillis() > timer && !running)
        {
            cached = null;
            running = true;
            timer = (long) (System.currentTimeMillis() + period);
        }
        if (cached == null)
        {
            cached = supplier.get();
            running = false;
        }
        return cached;
    }
}