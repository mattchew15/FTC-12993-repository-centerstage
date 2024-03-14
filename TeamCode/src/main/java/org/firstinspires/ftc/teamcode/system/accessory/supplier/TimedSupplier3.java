package org.firstinspires.ftc.teamcode.system.accessory.supplier;



import java.util.function.Supplier;
@Deprecated
public class TimedSupplier3<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private T cached = null;
    private double timeOut, previousTime;

    public double getTimeOut() {
        return timeOut;
    }

    public void setTimeOut(double timeOut) {
        this.timeOut = timeOut;
    }

    public TimedSupplier3(Supplier<T> supplier, double timeOut) {
        this.supplier = supplier;
        this.timeOut = timeOut;
        previousTime = System.nanoTime() / 1e9;
    }

    @Override
    public T get() {
        double currentTime = System.nanoTime() / 1e9;
        if (currentTime - previousTime > timeOut) {
            cached = null;
            previousTime = currentTime;
        }
        if (cached == null)
        {
            cached = supplier.get();
        }

        return cached;
    }
}