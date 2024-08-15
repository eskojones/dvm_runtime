package main

import (
	"fmt"
	"os"
)

const debug_mode = true

type interrupt_fn func(*context, uint8)

type context struct {
	stack      [1024]uint16
	registers  [16]uint16
	memory     [65536]uint8
	flags      [8]uint8
	interrupts [256]interrupt_fn
}

type ContextFlag uint8

const (
	flag_zero     ContextFlag = 0
	flag_lessthan ContextFlag = 1
	flag_morethan ContextFlag = 2
	flag_halt     ContextFlag = 3
)

type ContextRegister uint8

const (
	reg_pc ContextRegister = 15
	reg_sp ContextRegister = 14
)

type instr_fn func(*context) int

type instr struct {
	name   string
	opcode uint8
	size   uint8
	fn     instr_fn
}

// Globals
var instructions = make(map[uint8]instr) //map opcode->instruction
var op = make(map[string]uint8)          //map name->opcode

func startup() {
	// NOP - No Operation
	loadInstruction(instr{"nop", 0x00, 1, (func(ctx *context) int {
		return 0
	})})

	// HLT - Halt Program
	loadInstruction(instr{"hlt", 0xff, 1, (func(ctx *context) int {
		fmt.Printf("[i] halting execution\n")
		ctx.flags[flag_halt] = 1
		return 0xfe
	})})

	// PRINT - Print the value of register to debug output
	loadInstruction(instr{"print", 0xfe, 2, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value := ctx.registers[reg_index]
		fmt.Printf("[i] 0x%04x       R%d = 0x%04x\n", ctx.registers[reg_pc], reg_index, value)
		return 0
	})})

	// LD   reg,mem     - Load Register from Memory
	loadInstruction(instr{"ld", 0x01, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		mem_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		mem_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		mem_address := (mem_highbyte << 8) + mem_lowbyte
		mem_value := uint16(ctx.memory[mem_address])<<8 + uint16(ctx.memory[mem_address+1])
		// fmt.Printf("[i] load R%d <= 0x%04x (0x%04x)\n", reg_index, mem_address, mem_value)
		ctx.registers[reg_index] = mem_value
		return 0
	})})

	// ST   mem,reg     - Store Register to Memory
	loadInstruction(instr{"st", 0x02, 4, (func(ctx *context) int {
		mem_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+1])
		mem_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		mem_address := (mem_highbyte << 8) + mem_lowbyte
		reg_index := ctx.memory[ctx.registers[reg_pc]+3]
		reg_value_highbyte := uint8(ctx.registers[reg_index] >> 8)
		reg_value_lowbyte := uint8(ctx.registers[reg_index] & 0x00ff)
		// fmt.Printf("[i] store 0x%04x <= R%d (0x%02x%02x)\n", mem_address, reg_index, reg_value_highbyte, reg_value_lowbyte)
		ctx.memory[mem_address] = reg_value_highbyte
		ctx.memory[mem_address+1] = reg_value_lowbyte
		return 0
	})})

	// INT  reg       - Call Interrupt <value in reg>
	loadInstruction(instr{"int", 0x03, 2, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		int_num := uint8(ctx.registers[reg_index] & 0x00ff)
		if ctx.interrupts[int_num] != nil {
			ctx.interrupts[int_num](ctx, int_num)
		}
		return 0
	})})

	// INT  value       - Call Interrupt <value>
	loadInstruction(instr{"inti", 0x04, 3, (func(ctx *context) int {
		int_num := ctx.memory[ctx.registers[reg_pc]+2]
		if ctx.interrupts[int_num] != nil {
			ctx.interrupts[int_num](ctx, int_num)
		}
		return 0
	})})

	// RET              - Pop PC off of callstack and jump to address
	loadInstruction(instr{"ret", 0x05, 1, (func(ctx *context) int {
		if ctx.registers[reg_sp] == 0 {
			fmt.Printf("[!] invalid stack depth!")
			return 0xff
		}
		ctx.registers[reg_sp]--
		ctx.registers[reg_pc] = ctx.stack[ctx.registers[reg_sp]]
		return 1
	})})

	// MOV  reg,reg     - Move register into register
	loadInstruction(instr{"mov", 0x06, 3, (func(ctx *context) int {
		dst_reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		src_reg_index := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[dst_reg_index] = ctx.registers[src_reg_index]
		return 0
	})})

	// MOVI reg,value   - Move value into register
	loadInstruction(instr{"movi", 0x07, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		value_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		ctx.registers[reg_index] = uint16(value_highbyte<<8 + value_lowbyte)
		return 0
	})})

	// CMP  reg,reg     - Compare two registers
	loadInstruction(instr{"cmp", 0x08, 3, (func(ctx *context) int {
		reg1_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg2_index := ctx.memory[ctx.registers[reg_pc]+2]
		reg1_value := ctx.registers[reg1_index]
		reg2_value := ctx.registers[reg2_index]
		var result int16 = int16(reg2_value) - int16(reg1_value)
		if result == 0 {
			ctx.flags[flag_zero] = 1
			ctx.flags[flag_morethan] = 0
			ctx.flags[flag_lessthan] = 0
		} else if result < 0 {
			ctx.flags[flag_zero] = 0
			ctx.flags[flag_morethan] = 1
			ctx.flags[flag_lessthan] = 0
		} else if result > 0 {
			ctx.flags[flag_zero] = 0
			ctx.flags[flag_morethan] = 0
			ctx.flags[flag_lessthan] = 1
		}
		return 0
	})})

	// CMPI reg,value   - Compare register with value
	loadInstruction(instr{"cmpi", 0x09, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg_value := ctx.registers[reg_index]
		mem_highbyte := ctx.memory[ctx.registers[reg_pc]+2]
		mem_lowbyte := ctx.memory[ctx.registers[reg_pc]+3]
		mem_address := uint16(mem_highbyte)<<8 + uint16(mem_lowbyte)
		mem_value_highbyte := ctx.memory[mem_address]
		mem_value_lowbyte := ctx.memory[mem_address+1]
		mem_value := uint16(mem_value_highbyte)<<8 + uint16(mem_value_lowbyte)
		var result int16 = int16(mem_value) - int16(reg_value)
		if result == 0 {
			ctx.flags[flag_zero] = 1
			ctx.flags[flag_morethan] = 0
			ctx.flags[flag_lessthan] = 0
		} else if result < 0 {
			ctx.flags[flag_zero] = 0
			ctx.flags[flag_morethan] = 1
			ctx.flags[flag_lessthan] = 0
		} else if result > 0 {
			ctx.flags[flag_zero] = 0
			ctx.flags[flag_morethan] = 0
			ctx.flags[flag_lessthan] = 1
		}
		return 0
	})})

	// JMP  reg         - Unconditional jump to address contained in register
	loadInstruction(instr{"jmp", 0x0a, 2, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_pc] = ctx.registers[reg_index]
		return 1
	})})

	// JMPI address     - Unconditional jump to address
	loadInstruction(instr{"jmpi", 0x0b, 3, (func(ctx *context) int {
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg_pc] = uint16(address_highbyte)<<8 + uint16(address_lowbyte)
		return 1
	})})

	// JZ   reg         - Jump if zero flag set, address contained in register
	loadInstruction(instr{"jz", 0x0c, 2, (func(ctx *context) int {
		if ctx.flags[flag_zero] == 0 {
			return 0
		}
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_pc] = ctx.registers[reg_index]
		return 1
	})})

	// JZI  address     - Jump if zero flag set
	loadInstruction(instr{"jzi", 0x0d, 3, (func(ctx *context) int {
		if ctx.flags[flag_zero] == 0 {
			return 0
		}
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg_pc] = uint16(address_highbyte)<<8 + uint16(address_lowbyte)
		return 1
	})})

	// JNZ  reg
	loadInstruction(instr{"jnz", 0x0e, 2, (func(ctx *context) int {
		if ctx.flags[flag_zero] == 0 {
			return 0
		}
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_pc] = ctx.registers[reg_index]
		return 1
	})})

	// JNZI address
	loadInstruction(instr{"jnzi", 0x0f, 3, (func(ctx *context) int {
		if ctx.flags[flag_zero] == 0 {
			return 0
		}
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg_pc] = uint16(address_highbyte)<<8 + uint16(address_lowbyte)
		return 1
	})})

	// JL   reg
	loadInstruction(instr{"jl", 0x10, 2, (func(ctx *context) int {
		if ctx.flags[flag_lessthan] == 0 {
			return 0
		}
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_pc] = ctx.registers[reg_index]
		return 1
	})})

	// JLI  address
	loadInstruction(instr{"jli", 0x11, 3, (func(ctx *context) int {
		if ctx.flags[flag_lessthan] == 0 {
			return 0
		}
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg_pc] = uint16(address_highbyte)<<8 + uint16(address_lowbyte)
		return 1
	})})

	// JG   reg
	loadInstruction(instr{"jg", 0x12, 2, (func(ctx *context) int {
		if ctx.flags[flag_morethan] == 0 {
			return 0
		}
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_pc] = ctx.registers[reg_index]
		return 1
	})})

	// JGI  address
	loadInstruction(instr{"jgi", 0x13, 3, (func(ctx *context) int {
		if ctx.flags[flag_morethan] == 0 {
			return 0
		}
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg_pc] = uint16(address_highbyte)<<8 + uint16(address_lowbyte)
		return 1
	})})

	// INC  reg
	loadInstruction(instr{"inc", 0x14, 2, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_index]++
		return 0
	})})

	// DEC  reg
	loadInstruction(instr{"dec", 0x15, 2, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		ctx.registers[reg_index]--
		return 0
	})})

	// ADD  reg,reg
	loadInstruction(instr{"add", 0x16, 3, (func(ctx *context) int {
		reg1_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg2_index := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg1_index] += ctx.registers[reg2_index]
		return 0
	})})

	// ADDI reg,value
	loadInstruction(instr{"addi", 0x17, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		value_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		ctx.registers[reg_index] += (value_highbyte << 8) + value_lowbyte
		return 0
	})})

	// SUB  reg,reg
	loadInstruction(instr{"sub", 0x18, 3, (func(ctx *context) int {
		reg1_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg2_index := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg1_index] -= ctx.registers[reg2_index]
		return 0
	})})

	// SUBI reg,value
	loadInstruction(instr{"subi", 0x19, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		value_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		ctx.registers[reg_index] -= (value_highbyte << 8) + value_lowbyte
		return 0
	})})

	// MUL  reg,reg
	loadInstruction(instr{"mul", 0x1a, 3, (func(ctx *context) int {
		reg1_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg2_index := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg1_index] *= ctx.registers[reg2_index]
		return 0
	})})

	// MULI reg,value
	loadInstruction(instr{"muli", 0x1b, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		value_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		ctx.registers[reg_index] *= (value_highbyte << 8) + value_lowbyte
		return 0
	})})

	// DIV  reg,reg
	loadInstruction(instr{"div", 0x1c, 3, (func(ctx *context) int {
		reg1_index := ctx.memory[ctx.registers[reg_pc]+1]
		reg2_index := ctx.memory[ctx.registers[reg_pc]+2]
		ctx.registers[reg1_index] /= ctx.registers[reg2_index]
		return 0
	})})

	// DIVI reg,value
	loadInstruction(instr{"divi", 0x1d, 4, (func(ctx *context) int {
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		value_highbyte := uint16(ctx.memory[ctx.registers[reg_pc]+2])
		value_lowbyte := uint16(ctx.memory[ctx.registers[reg_pc]+3])
		ctx.registers[reg_index] /= (value_highbyte << 8) + value_lowbyte
		return 0
	})})

	// CALL reg     - Push PC onto callstack and jump to address contained in register
	loadInstruction(instr{"call", 0x1e, 2, (func(ctx *context) int {
		if ctx.registers[reg_sp] == 1023 {
			fmt.Printf("[!] maximum callstack exceeded!")
			return 0xff
		}
		reg_index := ctx.memory[ctx.registers[reg_pc]+1]
		address := ctx.registers[reg_index]
		ctx.stack[ctx.registers[reg_sp]] = ctx.registers[reg_pc] + 2
		ctx.registers[reg_sp]++
		ctx.registers[reg_pc] = address
		return 1
	})})

	// CALL address     - Push PC onto callstack and jump to address
	loadInstruction(instr{"calli", 0x1f, 3, (func(ctx *context) int {
		if ctx.registers[reg_sp] == 1023 {
			fmt.Printf("[!] maximum callstack exceeded!")
			return 0xff
		}
		address_highbyte := ctx.memory[ctx.registers[reg_pc]+1]
		address_lowbyte := ctx.memory[ctx.registers[reg_pc]+2]
		address := (uint16(address_highbyte) << 8) + uint16(address_lowbyte)
		ctx.stack[ctx.registers[reg_sp]] = ctx.registers[reg_pc] + 3
		ctx.registers[reg_sp]++
		ctx.registers[reg_pc] = address
		return 1
	})})
}

func step(instructions map[uint8]instr, ctx *context) int {
	opcode := ctx.memory[ctx.registers[reg_pc]]
	instr, exists := instructions[opcode]
	if !exists {
		fmt.Printf("[!] 0x%04x invalid opcode 0x%02x\n", ctx.registers[reg_pc], opcode)
		return 0xff
	}

	if debug_mode {
		fmt.Printf("[.] 0x%04x %-5s ", ctx.registers[reg_pc], instr.name)
		for i := uint8(0); i < instr.size; i++ {
			fmt.Printf("%02x ", ctx.memory[ctx.registers[reg_pc]+uint16(i)])
		}
		fmt.Printf("\n")
	}

	ret := instr.fn(ctx)
	if ret == 0 {
		ctx.registers[reg_pc] += uint16(instr.size)
	} else if ret == 1 {
		// the instruction modified PC
		ret = 0
	}

	return ret
}

func loadInstruction(instruction instr) {
	instructions[instruction.opcode] = instruction
	op[instruction.name] = instruction.opcode
}

func setInterrupt(ctx *context, interrupt int, function interrupt_fn) {
	ctx.interrupts[interrupt] = function
}

func handleInterrupts(ctx *context, interrupt uint8) {
	switch interrupt {
	case 1: //print char in r0
		fmt.Printf("%c", uint8(ctx.registers[0]&0x00ff))
	case 2: //print char in address stored in r0
		fmt.Printf("%c", ctx.memory[ctx.registers[0]])
	case 3: //print chars at address stored in r0 until null reached
		for i := ctx.registers[0]; ctx.memory[i] != 0; i++ {
			fmt.Printf("%c", ctx.memory[i])
		}
	}
}

func loadProgram(ctx *context, program []uint8, offset uint16) {
	for index, value := range program {
		ctx.memory[offset+uint16(index)] = value
	}
}

func main() {
	startup()

	bytecode, err := os.ReadFile(os.Args[1])
	if err != nil {
		fmt.Printf("file not found\n")
		return
	}

	program := []uint8(bytecode[:])
	for _, b := range program {
		fmt.Printf("%02x ", b)
	}
	fmt.Printf("\n")

	// program := []uint8{
	// 	op["movi"],  0x00, 0x00, 0x00,
	// 	op["movi"],  0x05, 0x00, 0x45,
	// //0x0008:
	// 	op["addi"],  0x00, 0x00, 0x01,
	// 	op["print"], 0x00,
	// 	op["cmp"],   0x00, 0x05,
	// 	op["jli"],   0x00, 0x08,
	// 	op["hlt"],
	// }

	ctx := new(context)
	for i := 0; i < 256; i++ {
		setInterrupt(ctx, i, handleInterrupts)
	}
	for idx := range ctx.memory {
		ctx.memory[idx] = op["hlt"]
	}
	loadProgram(ctx, program, 0)

	ret := 0
	for ret == 0 {
		ret = step(instructions, ctx)
	}
}
