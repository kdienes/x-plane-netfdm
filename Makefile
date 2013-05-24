all: netfdm.xpl

clean:
	$(RM) *.so *.o *.xpl

netfdm.o: netfdm.cpp
	cc -c -g -fPIC -DLIN -I./SDK/CHeaders/XPLM -I./SDK/CHeaders/Widgets -I/usr/include/JSBSim -g -c -o $@ $^

netfdm.xpl: netfdm.o
	cc -shared -dynamiclib -fPIC -fpic -o $@ $^
