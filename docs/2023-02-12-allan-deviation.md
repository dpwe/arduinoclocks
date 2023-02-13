### [dpwe](https://github.com/dpwe) [ClockBlog](index.html)

## 2023-02-12
# Comparing clock stability with Allan Deviation

In my [previous post](2023-01-15-ds3231-emulator.html), I described how I was adapting an oven-controlled crystal oscillator (OCXO) to behave like an integrated Real-Time Clock (RTC) chip (the DS3231, which is a temperature-compensate crystal oscillator, TCXO) -- i.e., keeping track of the date and time, and providing an interface for setting the time.  Because I used the Adafruit RP2040 Feather, it also supports battery-backup, although because the OCXO is quite power-hungry, the battery is only to tide it over brief power disconnections.

Once the clock was working, I naturally wanted to be able to measure its accuracy.  I'd previously developed the [synchronizer](2022-03-20-synchronizer.html) program to calculate the accuracy of DS3231 RTCs against a GPS reference.  But I hadn't looked in detail at how this accuracy varied on the scale of days, and this is the kind of detail that ought to differentiate the OCXO.

### Measuring Clock Stability

It turns out that characterizing the accuracy of a clock isn't entirely straighforward.  A given clock will have some error (offset) compared to a stable reference, but that error will also likely change systematically with time (drift).  However, to a large extent, drift is not a terribly serious problem, as long as that drift is known and constant, since it would then be trivial to calculate the true time.

So what really matters in a clock is how stable its timekeeping is over different time scales.  The standard way of characterizing this is [Allan Variance](https://en.wikipedia.org/wiki/Allan_variance), developed in the 1960s when very-accurate atomic clocks were first being produced.  The idea is to measure the proportional error between success measurements of supposedly-equal intervals, and to plot this over a wide range of intervals.

At the shortest intervals Allan Variance (or its square-root, Allan Deviation) is typically dominated by the random noise in each time measurement.  Measuring over longer intervals (e.g. summing up 10 or 100 successive intervals) will typically smooth out independent noise in each measurement, leading to more consistent values (when measured by difference as a proportion of total interval).  However, as the intervals become very long, uncontrolled drift in the frequency of the clock will begin to dominate -- if your clock changes its properties by 1ppm per day, that will limit how similar two successive day-long intervals will be.

Thus, Allan Deviation plots, which are plotted on log-log axes, are typically U-shaped: At the smallest intervals, the proportional error drops as the independent errors in each measurement are smoothed out through averaging.  But at some longer measurement interval, the underlying instability and drift of the clock frequency will begin to dominate, and this typically gets worse for longer intervals.

### My Allan Deviations

![Allan deviation plots](images/allan-deviation.png)

I managed to get about 370,000 seconds (about 4 days) of measurements from my logging Synchronizer before the system crashed.  This was enough to alllow me to get measurements of clock behavior, for both TXCO and OCXO (as well as the uncompensated system clock) out to 10^5 seconds.

I did all these calculations in a [Colab](https://colab.research.google.com/drive/1sQLEhoHQcFkfOhnLrahKhTjQoga9geAH), an online version of Python notebook.


