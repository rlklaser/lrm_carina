#!/usr/bin/r

# ROS
source(paste(system("rospack find rosR",intern=TRUE),"/lib/ros.R",sep=""),chdir=TRUE)

ros.Init("R_node")

dist <- c(0.031, 0.035, 0.039, 0.05, 0.06, 0.071, 0.08, 0.09, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.25, 0.3, 0.4)
volt <- c(3.02, 2.98, 2.72, 2.34, 2.01, 1.76, 1.55, 1.39, 1.26, 1.06, 0.93, 0.82, 0.73, 0.65, 0.53, 0.43, 0.32)

sharp.data <- data.frame( dist, volt )
sharp.reg  <- lm( dist ~ poly(volt, 8), sharp.data )
sharp.pred <- function(volt) { predict(sharp.reg, data.frame(volt)) }

sharp.p <- sharp.pred(seq(0.3,3,0.1))
sharp.c <- predict(sharp.reg, data.frame(volt=seq(0.3, 3, 0.1)), interval="confidence")

subscription <- ros.Subscriber("/sharpGP2D120/Voltage", "std_msgs/Float32")
publication  <- ros.Publisher("/sharpGP2D120/Distance", "sensor_msgs/Range")

range <- ros.Message("sensor_msgs/Range")
range$min_range <- 0.03
range$max_range <- 0.4
range$header$seq 		<- 0
range$header$frame_id	<- "/sharpGP2D120"

# R
x11(width=10, height=4, title='SharpGP2D120-Monitor')
layout(matrix(c(1,2), 1, 2, byrow = TRUE), widths=c(2,1), heights=c(1,1))
Distance <- Time <- rep(NA,100)

# run
while(ros.OK()){ # evaluates to TRUE as long as the master online
	ros.SpinOnce()
	if(ros.SubscriberHasNewMessage(subscription)){
		
		volt <- ros.ReadMessage(subscription)$data
		dist <- sharp.pred(volt)
		
		Distance <- c(Distance[-1], dist)
		Time  <- c(Time[-1],  ros.TimeNow())
		
		# R - Plotting
		plot(Time, Distance, t='l', main="Distance Measurements")
		lines(Time, filter(Distance, rep(0.1, 10), sides=1), type="l",col="blue", lwd=2.5)
		
		matplot(seq(0.3, 3, 0.1), cbind(sharp.p, sharp.c[,-1]), lty=c(1,2,2,3,3), col=c(1,2,2,3,3), type="l", xlab='Voltage', ylab='Distance', main='Linearization')
		points(volt, dist)
		
		# ROS
		range$range 		<- dist
#		range$header$stamp	<- ros.TimeNow()
		range$header$seq	<- range$header$seq + 1
		
		ros.WriteMessage(publication, range)
	}
}
