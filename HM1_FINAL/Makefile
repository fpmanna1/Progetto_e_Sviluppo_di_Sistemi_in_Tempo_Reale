all: plant controller reference replica diag

replica: replica.o rt-lib.o
	gcc -o replica replica.o rt-lib.o -lrt -pthread
plant: plant.o rt-lib.o
	gcc -o plant plant.o rt-lib.o -lrt -pthread

controller: controller.o rt-lib.o
	gcc -o controller controller.o rt-lib.o -lrt -pthread

diag: diag.o rt-lib.o
	gcc -o diag diag.o rt-lib.o -lrt -pthread

reference: reference.c parameters.h
	gcc -o reference reference.c -lrt

replica.o: replica.c rt-lib.h parameters.h
	gcc -c replica.c
plant.o: plant.c rt-lib.h parameters.h
	gcc -c plant.c

controller.o: controller.c rt-lib.h parameters.h
	gcc -c controller.c

diag.o: diag.c rt-lib.h parameters.h
	gcc -c diag.c

rt-lib.o: rt-lib.c rt-lib.h
	gcc -c rt-lib.c

clean:
	rm *.o plant controller reference replica  diag
