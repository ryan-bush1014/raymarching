build:
	gcc -Wall \
		-std=c99 \
		./*.c \
		-lm \
		-ggdb3 \
		-o main
