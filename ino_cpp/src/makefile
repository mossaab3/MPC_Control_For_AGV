#this file can be executed in case you face problems with the command "rosrun ino_cpp main.cpp"
ino_cpp: ino_cpp.cpp MPC.cpp ref.cpp MPC.h
	g++ -g ino_cpp.cpp MPC.cpp ref.cpp -o ino_cpp -I/opt/ros/melodic/include -L/opt/ros/melodic/lib \
	-Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime \
	-lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib














