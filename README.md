# Simulation_of_robot

Для запуска проекта: 
Cкопируйте файлы в репозиторий catkin_ws/src.
Запустите в терминале в этом порядке следующие команды.\n
$ cd ~catkit_ws
$ catkin_make --only-pkg-with-deps simulation_of_robot
$ roscore
$ rosrum simulation_of_robot robot.py
 
В новом терминале
$ rosrun simulation_of_robot server.py
  
После завершения программы появится график с реальной и целевой траекторией.
