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

***В окне robot.py будет выодиться***

Подтвержение получения от сервера целевой линейной и угловой скорости, и время получения.

`Getting from server linear velocity: 1.0(m/s)`

`Getting from server angular velicty: -0.5(rad/s)`

`Getting GTM time:                    2022-04-10 18:02:41.308885`

`Getting Unix epoch time:             1649602961308919191`


Значение левого и правого энкодера робота, и время отправки.

`Sending to server left encoder:  2054`

`Sending to server right encoder: 1779`

`Dispatching GTM time:            2022-04-10 17:25:37.334246`

`Dispatching Unix epoch time:     1649600737334106206 `

Текущие координаты робота x, y, угол поворота и время отправки.

`Sending coordinates to server (debug)`

`x = 1.945682743397427(m)`

`y = -1.539186817778079(m)`

`Î¸ = -1.338166656146812(rad)`

`Dispathing GMT time:             2022-04-10 17:25:37.336405`

`Dispathing Unix epoch time:      1649600737336420059`


***В окне server.py будет выодиться***

Целевая линейная и угловая скорость для робота, и время отправки.

`Sending to robot linear velocity: 1(m/s)`

`Sending to robot angular velicty: -0.5(rad/s)`

`Dispatching GMT time :            2022-04-10 17:57:26.057873`

`Dispatching Unix epoch time:      1649602646057917118`


Подтвердение получения значения энкодеров от робота и время их получения.

`Geting from robot left encoder: 4102`

`Geting from robot rightencoder: 3553`

`Receiving time GMT:             2022-04-10 17:25:37.335047`

`Receiving Unix epoch time:      1649600737335066795`

