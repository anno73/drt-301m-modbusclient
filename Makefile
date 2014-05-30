
test: all
	./mbc 

all: mbc

git:
	git commit -a
	git push origin master

mbc: mbc.c
	gcc -Wall -std=gnu99 mbc.c -o mbc -lmodbus -lm
