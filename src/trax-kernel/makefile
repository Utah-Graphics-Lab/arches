make: riscv riscv-dissasm

.PHONY: make riscv riscv-dissasm clean

#-mno-relax 
riscv: main.cpp
	mkdir -p riscv
	riscv64-unknown-elf-g++ -mno-relax -nostartfiles -emain -Wstack-usage=512 -mabi=lp64f -Ofast -I ../../include ./main.cpp -o ./riscv/kernel

riscv-dissasm: riscv
	mkdir -p riscv
	riscv64-unknown-elf-objdump -d -x ./riscv/kernel > ./riscv/kernel.dump

clean:
	rm -f riscv/*
