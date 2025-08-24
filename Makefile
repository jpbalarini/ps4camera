ifeq ($(shell uname -m), arm64)
	CCFLAGS = -L /opt/homebrew/lib -I /opt/homebrew/include
else
	CCFLAGS = -L /usr/local/lib
endif

main: main.cpp
	g++ $(CCFLAGS) -luvc `pkg-config --cflags --libs opencv4` -o main main.cpp

ps4eye: ps4eye_standalone.cpp ps4_eye.cpp
	g++ $(CCFLAGS) -std=c++11 -lusb-1.0 `pkg-config --cflags --libs opencv4` -o ps4eye_standalone ps4eye_standalone.cpp ps4_eye.cpp
