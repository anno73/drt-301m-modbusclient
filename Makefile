
test: all
	./mbc 

all: mbc

git: git_commit git_push

git_commit:
	git commit -a

git_push:
	git push origin master

mbc: mbc.c
	gcc -Wall -std=gnu99 mbc.c -o mbc -lmodbus -lm
