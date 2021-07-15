# Real Time Critical Power on the fly

I was puzzled by a publication that claimed to “Measure your FTP in a minute” in BikeRadar (https://www.bikeradar.com/advice/fitness-and-training/measure-your-ftp-in-a-minute-app-claims/?image=4&type=gallery&gallery=1&embedded_slideshow=1). Further Googling revealed that baronbiosys.com developed an <b>Xert-app</b> that claims this:

> The method uses sophisticated techniques and pattern recognition to determine your FTP. Whereas in the past you either needed to test using a 20 minute FTP protocol for example, or examine many months’ worth of data to get a realistic FTP value, this method enables you to determine your FTP on that day or even at that moment. (cf https://baronbiosys.com/real-time-ftp-determination/)
> 
After installing the <b>Xert app</b> on my Garmin (830), I have experienced, until free trial time was expended, that it does its estimates of <b>FTP</b> remarkably well. DcRainmaker reviewed it and is quite positive about its performance and accuracy and concluded:
> Whereas here it is the fact that I am getting FTP feedback in real-time that is so unique.  I can go out for a ride and start to see these values formulate as I am giving hard efforts.  No waiting hours, or even minutes later.  I am not aware of any other platform or app that does that. (cf https://www.dcrainmaker.com/2017/07/xert-rolls-out-free-real-time-ftp-app-on-garmin-devices.html) 
> 
I decided to develop C++ code for indoor training application that is running on an <b>Arduino nRF52</b> board and that comes as close as possible to the functionality of the <b>Xert app</b> (for Garmin Connect). The following is an explanation of the science and math behind its fundamentals. <br>

It was clear to me that the Xert app is based on depletion of the so-called <b>Anaerobic Work Capacity</b> (<b>AWC</b>) or <b>Functional Reserve Capacity</b> (<b>FRC</b>). To keep it simple, assume that a cyclist has a given amount of this <ins>finite work capacity</ins>) (energy) stored internally at the beginning of a ride. This amount of energy is known in mathematical terms as <b>W’</b> (pronounce <b>W prime</b>), it is energy and measured in joules. While you are riding at a low intensity, <b>W’</b> remains at its full level since it is not expended, and you are able to continue riding at this intensity for a long time. But if you push harder, you will start using this energy. The limit at which you will start expending this energy reserve is known as <b>Critical Power</b> (<b>CP</b>). If you push on the pedals harder than <b>CP</b>, <b>W’</b> will decrease. As soon as your produced power (watts) get lower than <b>CP</b>, <b>W’</b> will “regenerate”, and the energy reserve will increase again. When you ride long enough below <b>CP</b>, <b>W’</b> will approach the 100% level again. However, when you work hard and long enough above <b>CP</b>, <b>W’</b> will be depleted completely and you will be exhausted at the very moment (<b>Tlimit</b>)! All the variations in <b>W’</b> are expressed as “<b>W’ Balance</b>” in the Dr. Skiba algorithm (2). Another parameter of the algorithm is <b>Tau</b> that defines the speed at which <b>W’</b> is regenerating when the power is below <b>CP</b>. <br>

<b>CP</b> is defined as the highest exercise intensity that can be maintained for prolonged periods of time, typically for 45 to 60 min. Functional Threshold Power (<b>FTP</b>) is more well-known in recreational cycling and has been defined as the highest average power output that can be maintained for 60 min (1). Given the great similarity in definition, we assume for non-elite cycling non-significant differences between <b>CP</b> and <b>FTP</b> values! <br>

# It is all about algorithms <br>
The algorithms that had to be implemented are the original Dr. Skiba algorithm (2) and an optimization (approximation) of the Integral Skiba algorithm by Dave Waterworth (3). Aart Goossens published code (in Python) and explanatory information on Github (4) that helped enormously to understand and implement the different algorithms in an Arduino setting. The following information is paraphrased from his original work, to give the reader some insight in the mathematical background of the algorithms, see his work at: https://github.com/AartGoossens/publications/blob/master/w_balance_algorithms/article.ipynb <br>
# Integral Skiba algorithm <br>
The integral Skiba algorithm is the best-known algorithm to calculate <b>W' balance</b> and has been scientifically validated (5). The equations for the algorithm are: <br>
<img src="../main/images/W_Prime_equations.png" width="537" height="223" align = "middle" alt="W Prime equations"> <br>
Where <b>W'<sub>bal(t)</sub></b> is equal to <b>W'<sub>bal</sub></b> at time <b>t</b>, <b>W'</b> is the amount of available energy above <b>CP</b> (Critical Power), <b>t</b> the time for which <b>W'<sub>bal</sub></b> is calculated, <b>u</b> the iterator of the summation, <b>W'<sub>exp(u)</sub></b> amount of energy above <b>CP</b> that is used at time <b>u</b> (expended), <b>e</b> the Euler number and </b>Ʈ<sub>W'</sub></b> (pronounced <b>Tau</b>) a time constant that describes the recovery speed. The numbers 546, -0.01 and 316 are determined experimentally in Skiba's original article and do not change between individuals. <b>D<sub>CP</sub></b> is the difference between <b>CP</b> and the average power of the intervals in which the power was below <b>CP</b>. <b>D<sub>CP<sub></b> can be calculated dynamically (the average until time <b>t</b>) or calculated once for the entire workout and used as a static value. Skiba recommends using a static value for <b>D<sub>CP<sub></b>. <b>P(t)</b> is the power produced at time <b>t</b>.

# Dave Waterworth optimization of integral Skiba algorithm <br>
Mathematician Dave Waterworth (3) helped the core developer of Golden Cheetah (http://www.goldencheetah.org/), Mark Liversedge, to develop an optimization of the Skiba algorithm (6). This reformulation approximates the Skiba algorithm so results can vary a little in extreme cases only, especially when (<b>Tau</b>) is very small compared with the sample time.
The <b>W'<sub>bal</sub></b> integral part of the equations of Skiba is rewritten by Waterworth to: <br>
<img src="../main/images/Waterworth_equations.png" width="355" height="160" align = "middle" alt="Waterworth equations"> <br>
Where <b>S(t)</b> is a running sum at time t after the start, other symbols conform the previous equations. <b>Tau</b> (Ʈ<sub>W'</sub>) and <b>W'<sub>exp(t)</sub></b> are calculated with the original equations presented by Skiba. The integral Skiba algorithm is quite expensive to compute, even on fast computers since the summation must be repeated for every time <b>t</b> again. 
The big advantage of the Waterworth optimization is that now <b>W' balance</b> can be calculated at real time: during the ride and not only afterwards! In addition it is very helpful when one wants to determine the <b>Critical Power</b> on the fly during HIIT workouts or strenuous workouts when <b>W' balance</b> becomes negative and has been depleted!

# The Power-Duration Relationship <br>
<img src="../main/images/2-Parameter-Algorithm.png" width="800" height="400" align = "middle" alt="Power Duration"> <br>
Mathematically, the Power-Duration relationship is described as a hyperbolic function. The 4 different points on the curve represent points in Time (<b>T<sub>limit</sub></b>) when corresponding maximum sustainable Power above <b>CP</b> is reached and exhaustion occurs. When exercise tolerance is considered, the power-asymptote is known as <b>CP</b> (Watts). The curvature constant is known as <b>W'</b> (i.e., <b>W prime</b>), it is measured in units of work done (Joules). Notice that the 4 greyed areas, representing <b>W'</b>, are different in form but about equal in size. This hyperbolic power-duration relationship can be transformed into a linear relationship if work done is plotted against time, such that the slope of the line equals <b>CP</b> and the intercept equals <b>W'</b>. It should be emphasised that the power-duration relationship describes exercise tolerance but does not explain it. Nevertheless, the physiological responses to exercise performed below and above <b>CP</b> may provide important insights into the fatigue process. <b>CP</b> was originally defined as the external power output that could be sustained “indefinitely” or for a very long time without fatigue. This definition should be considered theoretical, however, since no exercise can ever be undertaken indefinitely. It is now understood that <b>CP</b> separates power outputs for which exercise tolerance is predictably limited (exercise power > <b>CP</b>). The actual time to intolerance (<b>T<sub>limit</sub></b>) for exercise performed above <b>CP</b> is defined, and therefore closely predicted, by the equation: <br><br>
   <strong>T<sub>lim</sub> = W′/(P-CP)</strong> <br><br>
This equation highlights that the time to intolerance above CP is a function of the proximity of the power output (<b>P</b>) being sustained to <b>CP</b> and the size of <b>W'</b>. When <b>P</b> is considerably above <b>CP</b>, the constant amount of work represented by the <b>W'</b> parameter will be utilized rapidly and <b>T<sub>lim</sub></b> will be short. Should <b>P</b> be closer to <b>CP</b>, then <b>W'</b> would be ‘used’ more slowly and <b>T<sup>lim</sup></b> would be longer. A crucial consideration here is that <b>W'</b> is assumed to be constant for all <b>P</b> above <b>CP</b>. This ‘<b>two parameter</b>’ power-time or power-duration model therefore implies that absolute exercise performance depends on simply the value of <b>CP</b> (in Watts) and the value of <b>W'</b> (in Joules). Both <b>CP</b> and <b>W'</b> parameters can vary considerably among individuals as a function of health/disease, age, fitness, and training.

1. Hunter A, Coggan A. Training and Racing with a Power Meter. Boulder (CO): Velo Press; 2010. p. 5-20, 41-65 p. 9
2. Skiba, P. F., Chidnok, W., Vanhatalo, A., & Jones, A. M. (2012). Modelling the expenditure and reconstitution of work capacity above critical power. Medicine and science in sports and exercise, 44(8), 1526-1532.
3. http://markliversedge.blogspot.nl/2014/10/wbal-optimisation-by-mathematician.html
4. https://github.com/AartGoossens/publications/blob/master/w_balance_algorithms/article.ipynb
5. Skiba, P. F., Jackman, S., Clarke, D., Vanhatalo, A., & Jones, A. M. (2014). Effect of work and recovery durations on W' reconstitution during intermittent exercise. Medicine and science in sports and exercise, 46(7), 1433-1440.
6. https://github.com/GoldenCheetah/GoldenCheetah/blob/master/src/Metrics/WPrime.cpp



