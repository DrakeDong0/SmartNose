## Project Description
Rotten food detector made by Ivan K. and Drake D. for our ECE 198 Project, November 22, 2023

This project is built with a STM32F401RE microcontroller, MQ Sensors, and a 1602A LCD Display and programmed in C. 

Upon start, the sensors are warmed up, then used to collect air quality data over a 10 second period. The methane, hydrogen sulfide and air quality data, in PPM, is then stored in a database, where it is then sorted using insertion sort.
The mean, median, and standard deviation are then displayed onto the LCD Display. The Z-score is calculated to replace outlier data with the median, and the measurement of error is used to determine if the data sample is precise enough.
## Pictures

### Start

![Start](Screenshots/IMG_4891.jpg)


