# Simulation_of_robot
Управление роботом при помощи ROS.
# Для запуска проекта: 

Cкопируйте файлы в репозиторий `catkin_ws/src`.
Запустите в терминале в этом порядке следующие команды.

`$ cd ~/catkin_ws`

`$ catkin_make --only-pkg-with-deps simulation_of_robot`

`$ roscore`

`$ rosrun simulation_of_robot robot.py`
 
В новом терминале

`$ rosrun simulation_of_robot server.py`
  
После завершения программы появится график с реальной и целевой траекторией.

В окне robot.py будет быводиться

Sending to server left encoder:  2054
Sending to server right encoder: 1779 
Dispatching GTM time:            2022-04-10 17:25:37.334246
Dispatching Unix epoch time:     1649600737334106206 
