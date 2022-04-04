#get data
setwd("~/r_files")
library(readODS)
true_stomp = read_ods(path = "dados_stomp.ods")
planning_time_stomp = read_ods(path = "dados_stomp.ods", sheet = 2)
trajectory_length_stomp = read_ods(path = "dados_stomp.ods", sheet = 3)
true_ompl = read_ods(path = "dados_ompl.ods")
planning_time_ompl = read_ods(path = "dados_ompl.ods", sheet = 2)
trajectory_length_ompl = read_ods(path = "dados_ompl.ods", sheet = 3)
true_chomp = read_ods(path = "dados_chomp.ods")
planning_time_chomp = read_ods(path = "dados_chomp.ods", sheet = 2)
trajectory_length_chomp = read_ods(path = "dados_chomp.ods", sheet = 3)

true_stomp[1] <- NULL
true_ratio_stomp = apply(true_stomp,2,function(x) sum(x == TRUE)/length(x) * 100)
true_ratio_stomp = mean(true_ratio_stomp)

true_ompl[1] <- NULL
true_ratio_ompl = apply(true_ompl,2,function(x) sum(x == TRUE)/length(x) * 100)
true_ratio_ompl = mean(true_ratio_ompl)

true_chomp[1] <- NULL
true_ratio_chomp = apply(true_chomp,2,function(x) sum(x == TRUE)/length(x) * 100)
true_ratio_chomp = mean(true_ratio_chomp)

planning_time_ompl[1] <- NULL
planning_time_stomp[1] <- NULL
planning_time_chomp[1] <- NULL
colnames(planning_time_ompl)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')
colnames(planning_time_stomp)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')
colnames(planning_time_chomp)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')
trajectory_length_ompl[1] <- NULL
trajectory_length_stomp[1] <- NULL
trajectory_length_chomp[1] <- NULL
colnames(trajectory_length_ompl)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')
colnames(trajectory_length_stomp)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')
colnames(trajectory_length_chomp)<-c('Traj1', 'Traj2', 'Traj3', 'Traj4', 'Traj5', 'Traj6')

library(tidyverse)
library(ggplot2)
shapiro.test(planning_time_ompl$`Traj6`)
shapiro.test(planning_time_ompl$`Traj5`)
shapiro.test(planning_time_ompl$`Traj4`)
hist(planning_time_ompl$`Traj1`)
hist(planning_time_ompl$`Traj6`)
hist(planning_time_ompl$`Traj5`)
shapiro.test(planning_time_stomp$`Traj6`)

traj6_planning_time = data_frame(planning_time_ompl$`Traj6`, planning_time_stomp$`Traj6`, planning_time_chomp$Traj6)
colnames(traj6_planning_time) <- c('OMPL', 'STOMP', 'CHOMP')
traj6_planning_time
boxplot(traj6_planning_time$OMPL, traj6_planning_time$STOMP, traj6_planning_time$CHOMP, 
        names = c('OMPL', 'STOMP', 'CHOMP'),
        main = "Planning Time (s) ",
        xlab = "Trajectory planner",
        ylab = "Time")

traj6_trajectory_length = data_frame(trajectory_length_ompl$`Traj6`, trajectory_length_stomp$`Traj6`, trajectory_length_chomp$Traj6)
colnames(traj6_trajectory_length) <- c('OMPL', 'STOMP', 'CHOMP')
traj6_trajectory_length
boxplot(traj6_trajectory_length$OMPL, traj6_trajectory_length$STOMP, traj6_trajectory_length$CHOMP, 
        names = c('OMPL', 'STOMP', 'CHOMP'),
        main = "Trajectory Duration (s) ",
        xlab = "Trajectory planner",
        ylab = "Time")


traj5_planning_time = data_frame(planning_time_ompl$`Traj5`, planning_time_stomp$`Traj5`, planning_time_chomp$Traj5)
colnames(traj5_planning_time) <- c('OMPL', 'STOMP', 'CHOMP')
traj5_planning_time
boxplot(traj5_planning_time$OMPL, traj5_planning_time$STOMP, traj5_planning_time$CHOMP, 
        names = c('OMPL', 'STOMP', 'CHOMP'),
        main = "Planning Time (s) ",
        xlab = "Trajectory planner",
        ylab = "Time")

traj5_trajectory_length = data_frame(trajectory_length_ompl$`Traj5`, trajectory_length_stomp$`Traj5`, trajectory_length_chomp$Traj5)
colnames(traj5_trajectory_length) <- c('OMPL', 'STOMP', 'CHOMP')
traj5_trajectory_length
boxplot(traj5_trajectory_length$OMPL, traj5_trajectory_length$STOMP, traj5_trajectory_length$CHOMP, 
        names = c('OMPL', 'STOMP', 'CHOMP'),
        main = "Trajectory Duration (s) ",
        xlab = "Trajectory planner",
        ylab = "Time")

