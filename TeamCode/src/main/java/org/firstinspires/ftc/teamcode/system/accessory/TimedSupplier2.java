package org.firstinspires.ftc.teamcode.system.accessory;


import java.util.function.Supplier;

public class TimedSupplier2<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private T cached = null;
    private double period, timer;

    public T getCached()
    {
        return cached;
    }

    public TimedSupplier2(Supplier<T> supplier, double period) {
        this.supplier = supplier;
        this.period = period;
        this.timer = System.currentTimeMillis();
    }

    @Override
    public T get() {
        if (System.currentTimeMillis() > timer) {
            cached = null;
            timer = System.currentTimeMillis() + period;
        }
        if (cached == null) cached = supplier.get();
        return cached;
    }
}