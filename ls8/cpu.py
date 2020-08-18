"""CPU functionality."""

import sys


class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.ram = [0] * 256
        self.reg = [0] * 7 + [0xF4]
        self.pc = 0

    def load(self):
        """Load a program into memory."""

        # Extract filename from command line
        try:
            filename = sys.argv[1]
            print(filename)
        except IndexError:
            print("Usage: python3 ls8.py <program_name>")
            sys.exit(1)

        # Validate filetype and confirm file exists
        if filename[-4:] != '.ls8':
            print("You must supply a '.ls8' binary.")
            sys.exit(2)
        try:
            f = open(filename)
        except FileNotFoundError:
            print(f"File not found: {filename}")
            sys.exit(2)

        # Read the contents of the file
        address = 0
        for line in f:
            try:
                opcode = line.split()[0]
            except IndexError:
                continue
            if opcode == '#':
                continue
            self.ram[address] = int(opcode, 2)
            address += 1
        f.close()

        # Double-check the file wasn't empty
        if address == 0:
            print("Error: Empty source file")
            sys.exit(2)

    def ram_read(self, addr):
        return self.ram[addr]

    def ram_write(self, addr, val):
        self.ram[addr] = val

    def alu(self, op):
        """ALU operations."""

        # Microcode
        def ADD():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.reg[regA] += self.reg[regB]
            self.reg[regA] &= 0xFF

        def AND():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.reg[regA] &= self.reg[regB]
            self.reg[regA] &= 0xFF

        def CMP():
            pass

        def DEC():
            pass

        def DIV():
            pass

        def INC():
            pass

        def MOD():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            if self.reg[regB] == 0:
                return True
            self.reg[regA] %= self.reg[regB]
            self.reg[regA] &= 0xFF

        def MUL():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.reg[regA] *= self.reg[regB]
            self.reg[regA] &= 0xFF

        def NOT():
            reg = self.ram[self.pc+1]
            self.reg[reg] = ~self.reg[reg]
            self.reg[regA] &= 0xFF

        def OR():
            regA = self.ram[self.pc+1]
            regB = self.ram[self.pc+2]
            self.reg[regA] |= self.reg[regB]
            self.reg[regA] &= 0xFF

        def SHL():
            pass

        def SHR():
            pass

        def SUB():
            pass

        def XOR():
            pass

        # Instruction mapping
        instructions = {
            0xA0: ADD,
            0xA8: AND,
            # 0xA7: CMP,
            # 0x66: DEC,
            # 0xA3: DIV,
            # 0x65: INC,
            0xA4: MOD,
            0xA2: MUL,
            0x69: NOT,
            0xAA: OR,
            # 0xAC: SHL,
            # 0xAD: SHR,
            # 0xA1: SUB,
            # 0xAB: XOR,
        }

        try:
            return instructions[op]()
        except KeyError:
            print(f"Unsupported ALU operation: {hex(op)}")
            return True  # This tells the CPU to halt

    def trace(self):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(f"TRACE: %02X | %02X %02X %02X |" % (
            self.pc,
            # self.fl,
            # self.ie,
            self.ram_read(self.pc),
            self.ram_read(self.pc + 1),
            self.ram_read(self.pc + 2)
        ), end='')

        for i in range(8):
            print(" %02X" % self.reg[i], end='')

        print()

    def run(self):
        """Run the CPU."""

        # Microcode
        def CALL():
            pass

        def HLT():
            return True

        def INT():
            pass

        def IRET():
            pass

        def JEQ():
            pass

        def JGE():
            pass

        def JGT():
            pass

        def JLE():
            pass

        def JLT():
            pass

        def JMP():
            self.pc = self.ram[self.pc+1]

        def JNE():
            pass

        def LD():
            reg = self.ram[self.pc+1]
            addr = self.reg[self.pc+2]
            val = self.ram[addr]
            self.reg[reg] = val

        def LDI():
            addr = self.ram[self.pc + 1]
            val = self.ram[self.pc + 2]
            self.reg[addr] = val

        def NOP():
            pass

        def POP():
            pass

        def PRA():
            addr = self.ram[self.pc+1]
            val = self.reg[addr]
            print(chr(val))

        def PRN():
            addr = self.ram[self.pc + 1]
            val = self.reg[addr]
            print(val)

        def PUSH():
            pass

        def RET():
            pass

        def ST():
            pass

        # Instruction mapping
        instructions = {
            0x50: CALL,
            0x01: HLT,
            # 0x52: INT,
            # 0x13: IRET,
            # 0x55: JEQ,
            # 0x5A: JGE,
            # 0x57: JGT,
            # 0x59: JLE,
            # 0x58: JLT,
            0x54: JMP,
            # 0x56: JNE,
            0x83: LD,
            0x82: LDI,
            0x00: NOP,
            # 0x46: POP,
            0x48: PRA,
            0x47: PRN,
            # 0x45: PUSH,
            # 0x11: RET,
            # 0x84: ST,
        }

        # Execution loop
        self.pc = 0
        while True:
            ir = self.ram[self.pc]

            # Execute non-ALU op
            if ~(ir >> 5) & 1:
                try:
                    # Only HLT returns True
                    if instructions[ir]():
                        break
                # Handle invalid ops
                except KeyError:
                    print(f"Invalid opcode: {hex(ir)}")
                    sys.exit(4)

            # Execute ALU op
            else:
                # Failed ops return True
                if self.alu(ir):
                    break

            # If the instruction doesn't set the PC
            if ~(ir >> 4) & 1:
                # Increment it based on the number of operands
                self.pc += (ir >> 6) + 1
