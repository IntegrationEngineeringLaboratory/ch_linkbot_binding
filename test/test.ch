/* File: recordangleneg.ch
   Record a joint angle and time, plot the acquired data,
   with a negative slope */
#include <linkbot.h>
#include <chplot.h>

CLinkbotI robot=CLinkbotI("ZRG6");
//CLinkbotI robot;
double speed = 45;         // speed in 45 degrees/seconds 
double timeInterval = 0.1; // time interval in 0.1 second 
int numDataPoints;         // number of data points recorded
robotRecordData_t timedata, angledata, angledata2, angledata3; // recorded time and angles for joint 1
CPlot plot;                // plotting class
double radius = 1.75;
double offset = 3;
double angle, angle1;
angle1 = 5;
angle =20;

//robot.connect();
robot.resetToZero();

robot.blinkLED(0.1, 5);
robot.recordNoDataShift();
robot.enableRecordDataShift();
robot.recordDistanceBegin(
    timedata,
    angledata,
    radius,
    0.1);
//sleep(2);
robot.move(90, 90, 90);

robot.recordDistanceEnd(numDataPoints);


plot.mathCoord();
plot.title("Angles for joint 1 versus time");
plot.label(PLOT_AXIS_X, "time (seconds)");
plot.label(PLOT_AXIS_Y, "angle for joint1 (degrees)");
plot.data2DCurve(timedata, angledata, numDataPoints);
plot.plotting();
    



